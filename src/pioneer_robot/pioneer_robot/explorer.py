import json
import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
import tf2_ros

# ── Merge map cleaning ───────────────────────────────────────────────────────
MORPH_OPEN_K     = 3    # open kernel size (r=1 → 3x3); removes isolated noise
MORPH_OPEN_ITER  = 1
MORPH_DIL_K      = 5    # dilate kernel size (r=2 → 5x5); expands/merges cylinders
MORPH_DIL_ITER   = 3
MORPH_CLOSE_K    = 5    # close kernel size (r=2 → 5x5); fills holes in merged blobs
MORPH_CLOSE_ITER = 8
MORPH_CLEAN_K    = 3    # final open kernel size (r=1 → 3x3); smooths edges
MORPH_CLEAN_ITER = 7

# ── Arena / sweep config ──────────────────────────────────────────────────────
ARENA_HALF       = 7.5
WALL_MARGIN      = 1.2
SWEEP_X          = [6.0, 0.0, -3.0]
SWEEP_Y_LO       = -(ARENA_HALF - WALL_MARGIN)
SWEEP_Y_HI       =  (ARENA_HALF - WALL_MARGIN)

# ── Timing ────────────────────────────────────────────────────────────────────
STARTUP_DELAY    = 5.0
TICK_HZ          = 0.5
CLUSTER_PERIOD   = 2.0
GOAL_TIMEOUT     = 100.0
RETURN_WAIT      = 10.0   # seconds to wait at origin before waypoint run

# ── Object detection ──────────────────────────────────────────────────────────
MIN_CLUSTER_PTS   = 5
MIN_CLUSTER_SPAN  = 0.25   # m — min bounding-box span; filters trees (~10cm) and cones (~12cm)
MAX_CLUSTER_SPAN  = 1.2    # m — max bounding-box span; rejects wall blobs and large noise
CLUSTER_RADIUS    = 0.6
MERGE_DIST        = 2.0

# ── Circling ──────────────────────────────────────────────────────────────────
CIRCLE_RADIUS    = 1.2
CIRCLE_N         = 3
CIRCLE_DWELL     = 2.0   # seconds to stop and scan at each circle waypoint

# ── Retry limits ─────────────────────────────────────────────────────────────
MAX_SWEEP_RETRIES  = 50
MAX_CIRCLE_RETRIES = 3


class FrontierExplorer(Node):

    WAITING    = 'waiting'
    SWEEP      = 'sweep'
    POST_SWEEP = 'post_sweep'
    VISITING   = 'visiting'
    RETURN     = 'return'
    WAYPOINT   = 'waypoint'
    DONE       = 'done'

    def __init__(self):
        super().__init__('frontier_explorer',
                         parameter_overrides=[rclpy.parameter.Parameter(
                             'use_sim_time',
                             rclpy.parameter.Parameter.Type.BOOL, False)])

        self._nav     = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._tf_buf  = tf2_ros.Buffer()
        self._tf_lis  = tf2_ros.TransformListener(self._tf_buf, self)

        self._status_pub   = self.create_publisher(String, '/explore/status', 10)
        self._trigger_pub  = self.create_publisher(String, '/perception/trigger', 10)
        self._costmap_params = self.create_client(
            SetParameters, '/global_costmap/global_costmap/set_parameters')
        self._costmap_frozen = False

        _transient = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE)
        self._latest_slam_map = None
        self._latest_costmap  = None
        self._merged_pub = self.create_publisher(
            OccupancyGrid, '/merged_costmap', _transient)
        self.create_subscription(OccupancyGrid, '/map',
                                 lambda m: setattr(self, '_latest_slam_map', m),
                                 _transient)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap',
                                 lambda m: setattr(self, '_latest_costmap', m),
                                 _transient)

        self.create_subscription(LaserScan, '/scan',
                                 lambda m: setattr(self, '_latest_scan', m), 10)
        self.create_subscription(String, '/detections', self._detection_cb, 10)

        # State
        self._state        = self.WAITING
        self._ready        = False
        self._goal_active  = False
        self._goal_sent_t  = 0.0
        self._last_goal_ok = True
        self._latest_scan  = None
        self._last_cluster_t = 0.0

        # Sweep
        self._sweep_wps   = self._gen_sweep()
        self._sweep_idx   = 0
        self._sweep_fails = 0

        # Visit queue
        self._visit_queue = []
        self._visited     = []

        # Circle sub-state
        self._circle_wps         = []
        self._circle_idx         = 0
        self._circle_fails       = 0
        self._current_visit_x    = 0.0
        self._current_visit_y    = 0.0
        self._circle_dwell_until = 0.0   # monotonic time to stop dwelling
        self._arrived_at_circle  = False  # set by goal_result_cb on circle arrival

        # Detected markers (populated via /detections)
        self._marker_wps = []   # list of (label, x, y)

        # Return / waypoint phase
        self._return_arrived_t = 0.0
        self._waypoint_queue   = []

        self.create_timer(STARTUP_DELAY, self._on_ready)
        self.create_timer(1.0 / TICK_HZ, self._tick)

        self.get_logger().info('FrontierExplorer ready — waiting for Nav2/SLAM ...')

    # ── Sweep generation ──────────────────────────────────────────────────────

    def _gen_sweep(self):
        pts = []
        for i, x in enumerate(SWEEP_X):
            if i % 2 == 0:
                pts += [(x, SWEEP_Y_HI), (x, SWEEP_Y_LO)]
            else:
                pts += [(x, SWEEP_Y_LO), (x, SWEEP_Y_HI)]
        wps = []
        for i, (x, y) in enumerate(pts):
            nx, ny = pts[i + 1] if i + 1 < len(pts) else (0.0, 0.0)
            wps.append((x, y, math.atan2(ny - y, nx - x)))
        return wps

    # ── Startup ───────────────────────────────────────────────────────────────

    def _on_ready(self):
        if not self._ready:
            self._ready = True
            self._state = self.SWEEP
            self.get_logger().info('Startup delay done — starting 3-pass sweep')

    # ── Detection intake ──────────────────────────────────────────────────────

    def _detection_cb(self, msg: String):
        try:
            data  = json.loads(msg.data)
            label = data['label']
            x, y  = data['x'], data['y']
            for lbl, wx, wy in self._marker_wps:
                if lbl == label and math.hypot(x - wx, y - wy) < MERGE_DIST:
                    return
            self._marker_wps.append((label, x, y))
            self.get_logger().info(
                f'[WP] marker logged: {label} at ({x}, {y}) — '
                f'{len(self._marker_wps)} total')
        except Exception:
            pass

    # ── Main tick ─────────────────────────────────────────────────────────────

    def _tick(self):
        if not self._ready:
            return

        if self._goal_active:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._goal_sent_t > GOAL_TIMEOUT:
                self.get_logger().warn('Goal timed out — forcing advance')
                self._goal_active  = False
                self._last_goal_ok = False
            else:
                return

        if self._state == self.SWEEP:
            self._detect_objects()
            self._tick_sweep()
        elif self._state == self.POST_SWEEP:
            self._tick_post_sweep()
        elif self._state == self.VISITING:
            self._tick_visit()
        elif self._state == self.RETURN:
            self._tick_return()
        elif self._state == self.WAYPOINT:
            self._tick_waypoint()

    def _tick_sweep(self):
        if not self._last_goal_ok:
            self._sweep_fails += 1
            if self._sweep_fails < MAX_SWEEP_RETRIES:
                self.get_logger().info(
                    f'[SWEEP] retry {self._sweep_fails}/{MAX_SWEEP_RETRIES} '
                    f'for WP {self._sweep_idx}')
                x, y, yaw = self._sweep_wps[self._sweep_idx - 1]
                self._last_goal_ok = True
                self._send_goal(x, y, yaw)
                return
            else:
                self.get_logger().warn(
                    f'[SWEEP] giving up on WP {self._sweep_idx} after '
                    f'{MAX_SWEEP_RETRIES} retries')
                self._sweep_fails = 0

        self._last_goal_ok = True

        if self._sweep_idx < len(self._sweep_wps):
            x, y, yaw = self._sweep_wps[self._sweep_idx]
            self._sweep_idx  += 1
            self._sweep_fails = 0
            self.get_logger().info(
                f'[SWEEP] WP {self._sweep_idx}/{len(self._sweep_wps)}: ({x}, {y})')
            self._send_goal(x, y, yaw)
        else:
            self.get_logger().info('[SWEEP] done — merging costmap, returning to origin')
            self._freeze_global_costmap()
            self._sort_queue_nn()
            self._state = self.POST_SWEEP
            self._last_goal_ok = True
            self._send_goal(0.0, 0.0)

    def _tick_post_sweep(self):
        self.get_logger().info('[POST_SWEEP] at origin — starting visit phase')
        self._last_goal_ok = True
        self._state = self.VISITING

    def _tick_visit(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self._circle_wps:
            # ── handle goal failure ─────────────────────────────────────────
            if not self._last_goal_ok:
                self._circle_fails += 1
                self._arrived_at_circle = False
                if self._circle_fails < MAX_CIRCLE_RETRIES:
                    x, y, yaw = self._circle_wps[self._circle_idx - 1]
                    self._last_goal_ok = True
                    self._send_goal(x, y, yaw)
                    return
                else:
                    self.get_logger().warn('[VISIT] skipping unreachable circle point')
                    self._circle_fails = 0

            self._last_goal_ok = True
            self._circle_fails = 0

            # ── start dwell window on first tick after arrival ──────────────
            if self._arrived_at_circle and self._circle_dwell_until == 0.0:
                self._arrived_at_circle  = False
                self._circle_dwell_until = now + CIRCLE_DWELL
                self.get_logger().info('[VISIT] arrived — dwelling for perception')

            # ── publish triggers continuously during dwell ──────────────────
            if self._circle_dwell_until > 0.0:
                if now < self._circle_dwell_until:
                    t = String()
                    t.data = json.dumps({
                        'x': self._current_visit_x,
                        'y': self._current_visit_y
                    })
                    self._trigger_pub.publish(t)
                    return
                self._circle_dwell_until = 0.0  # dwell done

            # ── advance to next point or finish circle ──────────────────────
            if self._circle_idx < len(self._circle_wps):
                x, y, yaw = self._circle_wps[self._circle_idx]
                self._circle_idx += 1
                self._send_goal(x, y, yaw)
            else:
                self.get_logger().info('[VISIT] circle complete')
                self._circle_wps = []
                self._circle_idx = 0

        elif self._visit_queue:
            ox, oy = self._visit_queue.pop(0)
            self._visited.append((ox, oy))
            self._current_visit_x    = ox
            self._current_visit_y    = oy
            self._circle_dwell_until = 0.0
            self._arrived_at_circle  = False
            self.get_logger().info(f'[VISIT] circling object at ({ox:.1f}, {oy:.1f})')
            self._pub_status(f'VISITING {ox:.1f} {oy:.1f}')
            self._circle_wps   = self._gen_circle(ox, oy)
            self._circle_idx   = 0
            self._circle_fails = 0
            self._last_goal_ok = True
            x, y, yaw = self._circle_wps[self._circle_idx]
            self._circle_idx += 1
            self._send_goal(x, y, yaw)

        else:
            self.get_logger().info(
                '[DONE] all objects inspected — returning to origin')
            self._pub_status('RETURNING')
            self._return_arrived_t = 0.0
            self._state = self.RETURN
            self._last_goal_ok = True
            self._send_goal(0.0, 0.0)

    def _tick_return(self):
        now = self.get_clock().now().nanoseconds / 1e9

        if self._return_arrived_t == 0.0:
            self._return_arrived_t = now
            self.get_logger().info(
                f'[RETURN] at origin — waiting {RETURN_WAIT:.0f}s before waypoint run')
            self._pub_status('AT_ORIGIN')
            return

        if now - self._return_arrived_t < RETURN_WAIT:
            return

        n = len(self._marker_wps)
        self.get_logger().info(
            f'[WAYPOINT] starting fast run to {n} markers')
        self._waypoint_queue = [(x, y) for _, x, y in self._marker_wps]
        self._sort_wps_nn(0.0, 0.0)
        self._last_goal_ok = True
        self._state = self.WAYPOINT

    def _tick_waypoint(self):
        if not self._last_goal_ok:
            self.get_logger().warn('[WAYPOINT] goal failed — skipping')
        self._last_goal_ok = True

        if self._waypoint_queue:
            x, y = self._waypoint_queue.pop(0)
            self.get_logger().info(f'[WAYPOINT] → ({x:.1f}, {y:.1f})')
            self._pub_status(f'WAYPOINT {x:.1f} {y:.1f}')
            self._send_goal(x, y)
        else:
            self.get_logger().info('[DONE] waypoint run complete')
            self._pub_status('DONE')
            self._state = self.DONE

    # ── Nearest-neighbour ordering ────────────────────────────────────────────

    def _sort_queue_nn(self):
        if len(self._visit_queue) <= 1:
            return
        rx, ry, _ = self._robot_pose()
        if rx is None:
            return
        self._visit_queue = self._nn_order(self._visit_queue, rx, ry)
        self.get_logger().info(
            f'[VISIT] order: {[(round(x,1), round(y,1)) for x,y in self._visit_queue]}')

    def _sort_wps_nn(self, cx: float, cy: float):
        if len(self._waypoint_queue) <= 1:
            return
        self._waypoint_queue = self._nn_order(self._waypoint_queue, cx, cy)
        self.get_logger().info(
            f'[WAYPOINT] order: {[(round(x,1),round(y,1)) for x,y in self._waypoint_queue]}')

    def _nn_order(self, pts, cx, cy):
        remaining = list(pts)
        ordered   = []
        while remaining:
            nearest = min(remaining, key=lambda p: math.hypot(p[0] - cx, p[1] - cy))
            ordered.append(nearest)
            remaining.remove(nearest)
            cx, cy = nearest
        return ordered

    # ── Costmap freeze + merge ────────────────────────────────────────────────

    def _freeze_global_costmap(self):
        if self._costmap_frozen:
            return
        if not self._costmap_params.service_is_ready():
            self.get_logger().warn('[COSTMAP] freeze service not ready — skipping')
            return
        self._merge_and_publish()
        req = SetParameters.Request()
        req.parameters = [
            Parameter(name='obstacle_layer.enabled',
                      value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                           bool_value=False)),
            Parameter(name='static_layer.enabled',
                      value=ParameterValue(type=ParameterType.PARAMETER_BOOL,
                                           bool_value=True)),
            Parameter(name='inflation_layer.inflation_radius',
                      value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                           double_value=0.375)),
        ]
        self._costmap_params.call_async(req)
        self._costmap_frozen = True
        self.get_logger().info('[COSTMAP] merged map published, switched to static layer')

    def _merge_and_publish(self):
        costmap = self._latest_costmap
        slam    = self._latest_slam_map
        if costmap is None:
            self.get_logger().warn('[MERGE] no costmap — skipping')
            return

        info = costmap.info
        w, h = info.width, info.height

        # Start from a blank grid — SLAM map drives everything, costmap ignored
        merged = np.zeros((h, w), dtype=np.int8)

        import cv2 as _cv2
        import os as _os
        save_dir = _os.path.expanduser('~/detections')
        _os.makedirs(save_dir, exist_ok=True)

        if slam is not None:
            si     = slam.info
            sw, sh = si.width, si.height
            sm     = np.array(slam.data, dtype=np.int8).reshape((sh, sw))

            # Save raw SLAM map before filtering
            slam_img = np.uint8(sm == 100) * 255
            _cv2.imwrite(_os.path.join(save_dir, 'map_1_slam_raw.png'), slam_img)

            # Filter SLAM map before merge
            def _ellipse(k): return _cv2.getStructuringElement(_cv2.MORPH_ELLIPSE, (k, k))
            slam_clean = _cv2.morphologyEx(slam_img, _cv2.MORPH_OPEN,   _ellipse(MORPH_OPEN_K),   iterations=MORPH_OPEN_ITER)
            slam_clean = _cv2.dilate      (slam_clean,                   _ellipse(MORPH_DIL_K),    iterations=MORPH_DIL_ITER)
            slam_clean = _cv2.morphologyEx(slam_clean, _cv2.MORPH_CLOSE, _ellipse(MORPH_CLOSE_K),  iterations=MORPH_CLOSE_ITER)
            slam_clean = _cv2.morphologyEx(slam_clean, _cv2.MORPH_OPEN,  _ellipse(MORPH_CLEAN_K),  iterations=MORPH_CLEAN_ITER)
            _cv2.imwrite(_os.path.join(save_dir, 'map_2_slam_filtered.png'), slam_clean)

            ratio  = si.resolution / info.resolution
            ox_off = (si.origin.position.x - info.origin.position.x) / info.resolution
            oy_off = (si.origin.position.y - info.origin.position.y) / info.resolution

            cols = np.arange(w)
            rows = np.arange(h)
            sc   = np.clip(np.floor((cols - ox_off) * ratio).astype(int), 0, sw - 1)
            sr   = np.clip(np.floor((rows - oy_off) * ratio).astype(int), 0, sh - 1)

            slam_overlay = slam_clean[np.ix_(sr, sc)]
            merged = np.where(slam_overlay == 255, np.int8(100), merged)
            self.get_logger().info('[MERGE] filtered SLAM merged in')
        else:
            self.get_logger().warn('[MERGE] no SLAM map — using costmap only')

        # Save final merged result
        merged_img = np.uint8(merged == 100) * 255
        _cv2.imwrite(_os.path.join(save_dir, 'map_3_merged_final.png'), merged_img)
        self.get_logger().info(f'[MERGE] maps saved to {save_dir}')
        self.get_logger().info(f'[MERGE] maps saved to {save_dir}')

        out = OccupancyGrid()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = 'map'
        out.info            = info
        out.data            = merged.flatten().tolist()
        self._merged_pub.publish(out)
        self.get_logger().info(f'[MERGE] published /merged_costmap ({w}x{h})')
        self._extract_objects_from_costmap(merged, info)

    def _extract_objects_from_costmap(self, merged, info):
        import cv2 as _cv2
        binary = np.uint8(merged == 100) * 255
        n, _, stats, centroids = _cv2.connectedComponentsWithStats(binary, connectivity=8)

        objects = []
        for i in range(1, n):
            area   = int(stats[i, _cv2.CC_STAT_AREA])
            bw     = int(stats[i, _cv2.CC_STAT_WIDTH])
            bh     = int(stats[i, _cv2.CC_STAT_HEIGHT])
            aspect = max(bw, bh) / max(min(bw, bh), 1)

            if area < 200 or area > 600:
                continue
            if aspect > 5.0:
                continue
            span_m = max(bw, bh) * info.resolution
            if span_m < MIN_CLUSTER_SPAN or span_m > MAX_CLUSTER_SPAN:
                continue

            cx_cell = centroids[i][0]
            cy_cell = centroids[i][1]
            wx = info.origin.position.x + cx_cell * info.resolution
            wy = info.origin.position.y + cy_cell * info.resolution

            if abs(wx) > ARENA_HALF - WALL_MARGIN or abs(wy) > ARENA_HALF - WALL_MARGIN:
                continue

            objects.append((wx, wy))

        if not objects:
            self.get_logger().warn(
                f'[MERGE] no objects found in merged costmap — '
                f'falling back to {len(self._visit_queue)} LiDAR detections')
            return

        self._visit_queue = objects
        self._visited     = []
        self.get_logger().info(
            f'[MERGE] visit queue replaced with {len(objects)} costmap centroids: '
            f'{[(round(x,1), round(y,1)) for x, y in objects]}')

    # ── Scan-based object detection ───────────────────────────────────────────

    def _detect_objects(self):
        if self._latest_scan is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self._last_cluster_t < CLUSTER_PERIOD:
            return
        self._last_cluster_t = now

        rx, ry, heading = self._robot_pose()
        if rx is None:
            return

        scan = self._latest_scan
        pts  = []
        ang  = scan.angle_min
        for r in scan.ranges:
            if scan.range_min < r < min(scan.range_max, 9.0):
                lx = r * math.cos(ang)
                ly = r * math.sin(ang)
                wx = rx + lx * math.cos(heading) - ly * math.sin(heading)
                wy = ry + lx * math.sin(heading) + ly * math.cos(heading)
                if (abs(wx) < ARENA_HALF - WALL_MARGIN and
                        abs(wy) < ARENA_HALF - WALL_MARGIN):
                    pts.append((wx, wy))
            ang += scan.angle_increment

        for cx, cy in self._cluster(pts):
            if self._is_new(cx, cy):
                self._visit_queue.append((cx, cy))
                self.get_logger().info(
                    f'[DETECT] new object queued at ({cx:.1f}, {cy:.1f})')

    def _cluster(self, pts):
        used = [False] * len(pts)
        out  = []
        for i, (px, py) in enumerate(pts):
            if used[i]:
                continue
            grp = [(px, py)]
            used[i] = True
            for j in range(i + 1, len(pts)):
                if not used[j]:
                    qx, qy = pts[j]
                    if math.hypot(px - qx, py - qy) < CLUSTER_RADIUS:
                        grp.append((qx, qy))
                        used[j] = True
            if len(grp) >= MIN_CLUSTER_PTS:
                xs = [p[0] for p in grp]
                ys = [p[1] for p in grp]
                span = max(max(xs) - min(xs), max(ys) - min(ys))
                if span < MIN_CLUSTER_SPAN:
                    continue
                cx = (max(xs) + min(xs)) / 2.0
                cy = (max(ys) + min(ys)) / 2.0
                out.append((cx, cy))
        return out

    def _is_new(self, x, y):
        for qx, qy in self._visit_queue + self._visited:
            if math.hypot(x - qx, y - qy) < MERGE_DIST:
                return False
        return True

    # ── Circle generation ─────────────────────────────────────────────────────

    def _gen_circle(self, ox, oy):
        # 180-degree arc on the side of the object facing the arena centre (0,0)
        # All signs face inward so this covers the sign face for every object
        inner      = ARENA_HALF - WALL_MARGIN
        to_centre  = math.atan2(-oy, -ox)   # direction from object toward (0,0)
        wps        = []
        for i in range(CIRCLE_N):
            ang = to_centre - math.radians(50) + math.radians(100) * i / (CIRCLE_N - 1)
            wx  = max(-inner, min(inner, ox + CIRCLE_RADIUS * math.cos(ang)))
            wy  = max(-inner, min(inner, oy + CIRCLE_RADIUS * math.sin(ang)))
            yaw = math.atan2(oy - wy, ox - wx)
            wps.append((wx, wy, yaw))
        return wps

    # ── Nav2 goal sending ─────────────────────────────────────────────────────

    def _send_goal(self, x, y, yaw=None):
        if not self._nav.wait_for_server(timeout_sec=0.0):
            self.get_logger().warn('Nav2 not ready', throttle_duration_sec=5.0)
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp    = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        if yaw is not None:
            goal.pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal.pose.pose.orientation.w = math.cos(yaw / 2.0)
        else:
            goal.pose.pose.orientation.w = 1.0

        self._goal_active = True
        self._goal_sent_t = self.get_clock().now().nanoseconds / 1e9
        fut = self._nav.send_goal_async(goal)
        fut.add_done_callback(self._goal_accepted_cb)

    def _goal_accepted_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Goal rejected by Nav2')
            self._last_goal_ok = False
            self._goal_active  = False
            return
        handle.get_result_async().add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result()
        self._last_goal_ok = (result.status == GoalStatus.STATUS_SUCCEEDED)
        self._goal_active  = False
        if self._state == self.VISITING and self._last_goal_ok:
            self._arrived_at_circle = True

    # ── TF helper ─────────────────────────────────────────────────────────────

    def _robot_pose(self):
        try:
            t   = self._tf_buf.lookup_transform('map', 'base_link', rclpy.time.Time())
            x   = t.transform.translation.x
            y   = t.transform.translation.y
            q   = t.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return x, y, yaw
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}', throttle_duration_sec=5.0)
            return None, None, None

    # ── Status publisher ──────────────────────────────────────────────────────

    def _pub_status(self, msg: str):
        s      = String()
        s.data = msg
        self._status_pub.publish(s)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
