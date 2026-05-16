import json
import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf2_ros

# ── Arena / sweep config ──────────────────────────────────────────────────────
ARENA_HALF       = 7.5
WALL_MARGIN      = 1.2
SWEEP_X          = [-6.0, 0.0, 6.0]
SWEEP_Y_LO       = -(ARENA_HALF - WALL_MARGIN)
SWEEP_Y_HI       =  (ARENA_HALF - WALL_MARGIN)

# ── Timing ────────────────────────────────────────────────────────────────────
STARTUP_DELAY    = 10.0
TICK_HZ          = 0.5
CLUSTER_PERIOD   = 2.0
GOAL_TIMEOUT     = 40.0
RETURN_WAIT      = 10.0   # seconds to wait at origin before waypoint run

# ── Object detection ──────────────────────────────────────────────────────────
MIN_CLUSTER_PTS   = 5
MIN_CLUSTER_SPAN  = 0.25   # m — min bounding-box span; filters trees (~10cm) and cones (~12cm)
CLUSTER_DEPTH_EST = 0.25   # m — half box depth; pushes bounding-box face centre to true centre
CLUSTER_RADIUS    = 0.6
MERGE_DIST        = 2.0

# ── Circling ──────────────────────────────────────────────────────────────────
CIRCLE_RADIUS    = 1.8
CIRCLE_N         = 8
CIRCLE_DWELL     = 2.0   # seconds to stop and scan at each circle waypoint

# ── Retry limits ─────────────────────────────────────────────────────────────
MAX_SWEEP_RETRIES  = 20
MAX_CIRCLE_RETRIES = 3


class FrontierExplorer(Node):

    WAITING   = 'waiting'
    SWEEP     = 'sweep'
    VISITING  = 'visiting'
    RETURN    = 'return'
    WAYPOINT  = 'waypoint'
    DONE      = 'done'

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
        wps = []
        for i, x in enumerate(SWEEP_X):
            if i % 2 == 0:
                wps += [(x, SWEEP_Y_LO), (x, SWEEP_Y_HI)]
            else:
                wps += [(x, SWEEP_Y_HI), (x, SWEEP_Y_LO)]
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
                x, y = self._sweep_wps[self._sweep_idx - 1]
                self._last_goal_ok = True
                self._send_goal(x, y)
                return
            else:
                self.get_logger().warn(
                    f'[SWEEP] giving up on WP {self._sweep_idx} after '
                    f'{MAX_SWEEP_RETRIES} retries')
                self._sweep_fails = 0

        self._last_goal_ok = True

        if self._sweep_idx < len(self._sweep_wps):
            x, y = self._sweep_wps[self._sweep_idx]
            self._sweep_idx  += 1
            self._sweep_fails = 0
            self.get_logger().info(
                f'[SWEEP] WP {self._sweep_idx}/{len(self._sweep_wps)}: ({x}, {y})')
            self._send_goal(x, y)
        else:
            n = len(self._visit_queue)
            self.get_logger().info(f'[SWEEP] done — {n} objects queued')
            self._sort_queue_nn()
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

        for cx, cy in self._cluster(pts, rx, ry):
            if self._is_new(cx, cy):
                self._visit_queue.append((cx, cy))
                self.get_logger().info(
                    f'[DETECT] new object queued at ({cx:.1f}, {cy:.1f})')

    def _cluster(self, pts, rx, ry):
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
                # bounding-box centre is more stable than point-cloud mean
                cx = (max(xs) + min(xs)) / 2.0
                cy = (max(ys) + min(ys)) / 2.0
                # LiDAR only sees the near face — push centroid toward true centre
                dx, dy = cx - rx, cy - ry
                d = math.hypot(dx, dy)
                if d > 0:
                    cx += (dx / d) * CLUSTER_DEPTH_EST
                    cy += (dy / d) * CLUSTER_DEPTH_EST
                out.append((cx, cy))
        return out

    def _is_new(self, x, y):
        for qx, qy in self._visit_queue + self._visited:
            if math.hypot(x - qx, y - qy) < MERGE_DIST:
                return False
        return True

    # ── Circle generation ─────────────────────────────────────────────────────

    def _gen_circle(self, ox, oy):
        rx, ry, _ = self._robot_pose()
        start = math.atan2((ry or 0.0) - oy, (rx or 0.0) - ox)
        inner = ARENA_HALF - WALL_MARGIN
        wps   = []
        for i in range(CIRCLE_N):
            ang = start + 2.0 * math.pi * i / CIRCLE_N
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
