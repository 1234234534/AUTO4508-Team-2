import os
import json
import math
import cv2
import numpy as np
from datetime import datetime

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# ── Colour thresholds (OpenCV HSV: H 0-179, S 0-255, V 0-255) ───────────────
RED_LO1  = np.array([  0, 120,  40], dtype=np.uint8)
RED_HI1  = np.array([ 10, 255, 255], dtype=np.uint8)
RED_LO2  = np.array([170, 120,  40], dtype=np.uint8)
RED_HI2  = np.array([179, 255, 255], dtype=np.uint8)
GREEN_LO = np.array([ 40,  80,  40], dtype=np.uint8)
GREEN_HI = np.array([ 85, 255, 255], dtype=np.uint8)
BLUE_LO  = np.array([100, 120,  40], dtype=np.uint8)
BLUE_HI  = np.array([130, 255, 255], dtype=np.uint8)

MIN_PIXELS      = 300    # px threshold per frame to count a colour hit
MIN_VOTE_FRAMES = 0      # minimum winning frame count before committing
CAPTURE_WINDOW  = 3.0    # seconds — window refreshed each trigger tick
COMMIT_SILENCE  = 4.0    # seconds without a trigger → commit accumulated votes
POSITION_CHANGE = 1.5    # m — trigger position shift that forces a commit + reset

# ── Geometry ─────────────────────────────────────────────────────────────────
DEDUP_RADIUS  = 3.0
ROI_TOP_FRAC  = 0.15
ROI_BOT_FRAC  = 0.35


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node',
                         parameter_overrides=[rclpy.parameter.Parameter(
                             'use_sim_time',
                             rclpy.parameter.Parameter.Type.BOOL, False)])

        self._bridge = CvBridge()
        self._latest_frame  = None
        self._trigger_until = 0.0
        self._last_trigger_t = 0.0

        # Current obstacle being voted on
        self._vote_x: float | None = None
        self._vote_y: float | None = None
        self._votes: dict[str, int] = {}   # label -> frame count this visit
        self._best_masks: dict[str, any] = {}  # label -> best mask seen so far

        self.create_subscription(Image, '/oak/rgb/image_raw', self._image_cb, 10)
        self.create_subscription(String, '/perception/trigger', self._trigger_cb, 10)
        self.create_timer(0.1, self._process)

        self._detection_pub = self.create_publisher(String, '/detections', 10)

        self._save_dir = os.path.expanduser('~/ros2_ws_part3/detections')
        os.makedirs(self._save_dir, exist_ok=True)
        self._log_path = os.path.join(self._save_dir, 'detections.json')

        self._logged: list[dict] = []   # all detections ever (written to file)
        self._session: list[dict] = []  # this run only — used for dedup
        if os.path.exists(self._log_path):
            try:
                with open(self._log_path) as f:
                    self._logged = json.load(f)
                self.get_logger().info(
                    f'Loaded {len(self._logged)} existing detections from log')
            except Exception:
                pass

        self.get_logger().info(f'PerceptionNode ready — saving to {self._save_dir}')

    # ── image intake ─────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}', throttle_duration_sec=5.0)

    def _trigger_cb(self, msg: String):
        import time
        try:
            data = json.loads(msg.data)
            x, y = data['x'], data['y']
        except Exception:
            return

        now = time.monotonic()

        # Obstacle changed — commit votes for the previous one before resetting
        if (self._vote_x is not None and
                math.hypot(x - self._vote_x, y - self._vote_y) > POSITION_CHANGE):
            self._commit_votes()

        self._vote_x = x
        self._vote_y = y
        self._trigger_until  = now + CAPTURE_WINDOW
        self._last_trigger_t = now

    # ── main processing loop ─────────────────────────────────────────────────

    def _process(self):
        import time
        now = time.monotonic()

        # Commit if trigger has gone silent (dwell ended, robot moved on)
        if (self._votes and self._vote_x is not None and
                self._last_trigger_t > 0.0 and
                now - self._last_trigger_t > COMMIT_SILENCE):
            self._commit_votes()
            return

        if self._latest_frame is None or now > self._trigger_until:
            return

        frame = self._latest_frame
        h, w  = frame.shape[:2]
        y0    = int(h * ROI_TOP_FRAC)
        y1    = int(h * (1.0 - ROI_BOT_FRAC))
        roi   = frame[y0:y1, :]
        hsv   = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_mask   = (cv2.inRange(hsv, RED_LO1, RED_HI1) |
                      cv2.inRange(hsv, RED_LO2, RED_HI2))
        green_mask = cv2.inRange(hsv, GREEN_LO, GREEN_HI)
        blue_mask  = cv2.inRange(hsv, BLUE_LO,  BLUE_HI)

        r_px = cv2.countNonZero(red_mask)
        g_px = cv2.countNonZero(green_mask)
        b_px = cv2.countNonZero(blue_mask)
        self.get_logger().info(
            f'[Perception] px — red:{r_px} green:{g_px} blue:{b_px} '
            f'(need {MIN_PIXELS})',
            throttle_duration_sec=0.5)

        for mask, label, px in [(red_mask, 'red', r_px),
                                 (green_mask, 'green', g_px),
                                 (blue_mask, 'blue', b_px)]:
            if px >= MIN_PIXELS:
                self._votes[label] = self._votes.get(label, 0) + 1
                # Keep the frame with the most pixels as the saved image
                if label not in self._best_masks or px > self._best_masks[label][1]:
                    self._best_masks[label] = (roi.copy(), px, mask.copy())

    # ── vote commit ──────────────────────────────────────────────────────────

    def _commit_votes(self):
        if not self._votes or self._vote_x is None:
            return

        winner = max(self._votes, key=self._votes.get)
        count  = self._votes[winner]
        self.get_logger().info(
            f'[Perception] votes at ({self._vote_x:.1f},{self._vote_y:.1f}): '
            f'{self._votes} → winner={winner} ({count} frames)')

        if count >= MIN_VOTE_FRAMES:
            frame, _, mask = self._best_masks[winner]
            self._save_detection(winner, frame, mask, self._vote_x, self._vote_y)
        else:
            self.get_logger().info(
                f'[Perception] vote rejected — winner only {count} frames '
                f'(need {MIN_VOTE_FRAMES})')

        self._votes      = {}
        self._best_masks = {}
        self._vote_x     = None
        self._vote_y     = None

    # ── save + publish ────────────────────────────────────────────────────────

    def _save_detection(self, label: str, frame, mask, ox: float, oy: float):
        mx = round(ox, 2)
        my = round(oy, 2)

        for existing in self._session:
            if (existing['label'] == label and
                    math.hypot(mx - existing['x'], my - existing['y']) < DEDUP_RADIUS):
                self.get_logger().info(
                    f'[Perception] dedup — {label} already logged near ({mx},{my})')
                return

        timestamp    = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:19]
        img_filename = f'{label}_{timestamp}.jpg'
        img_path     = os.path.join(self._save_dir, img_filename)

        annotated = frame.copy()
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(annotated, contours, -1, (0, 255, 0), 2)
        cv2.putText(annotated, f'{label} ({mx:.1f},{my:.1f})',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.imwrite(img_path, annotated)

        entry = {
            'label':     label,
            'category':  'marker',
            'x':         mx,
            'y':         my,
            'image':     img_path,
            'timestamp': timestamp,
        }
        self._session.append(entry)
        self._logged.append(entry)

        with open(self._log_path, 'w') as f:
            json.dump(self._logged, f, indent=2)

        self.get_logger().info(
            f'[Perception] MARKER logged: {label} at ({mx},{my}) — {img_filename}')

        out = String()
        out.data = json.dumps(entry)
        self._detection_pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
