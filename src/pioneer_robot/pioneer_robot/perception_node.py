import os
import json
import math
from datetime import datetime

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge

# ── HSV colour ranges (H 0-179, S 0-255, V 0-255) ────────────────────────────
RED_LO1 = np.array([  0, 120,  40], dtype=np.uint8)
RED_HI1 = np.array([ 10, 255, 255], dtype=np.uint8)
RED_LO2 = np.array([170, 120,  40], dtype=np.uint8)
RED_HI2 = np.array([179, 255, 255], dtype=np.uint8)
YEL_LO  = np.array([ 20, 100,  80], dtype=np.uint8)
YEL_HI  = np.array([ 35, 255, 255], dtype=np.uint8)

MIN_PIXELS  = 800    # minimum coloured pixels to count as a detection
DEDUP_DIST  = 3.0    # m — suppress duplicate detections within this radius
ROI_TOP     = 0.10   # crop top fraction of image (sky / ceiling)
ROI_BOT     = 0.20   # crop bottom fraction (ground)

SAVE_DIR = os.path.expanduser('~/ros2_ws_part3/detections')


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node',
                         parameter_overrides=[rclpy.parameter.Parameter(
                             'use_sim_time',
                             rclpy.parameter.Parameter.Type.BOOL, False)])

        self._bridge       = CvBridge()
        self._latest_frame = None
        self._session: list[dict] = []

        self.create_subscription(Image,  '/oak/rgb/image_raw',    self._image_cb,   10)
        self.create_subscription(String, '/perception/trigger',   self._trigger_cb, 10)
        self._det_pub = self.create_publisher(String, '/detections', 10)

        os.makedirs(SAVE_DIR, exist_ok=True)
        self._log_path = os.path.join(SAVE_DIR, 'detections.json')
        self.get_logger().info(f'PerceptionNode ready — saving to {SAVE_DIR}')

    # ── image intake ──────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image):
        try:
            self._latest_frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}', throttle_duration_sec=5.0)

    # ── trigger: one snapshot per dwell ──────────────────────────────────────

    def _trigger_cb(self, msg: String):
        if self._latest_frame is None:
            self.get_logger().warn('[PERCEPTION] trigger fired but no image yet')
            return
        try:
            data = json.loads(msg.data)
            ox, oy = float(data['x']), float(data['y'])
        except Exception:
            return

        frame  = self._latest_frame.copy()
        h, w   = frame.shape[:2]
        y0, y1 = int(h * ROI_TOP), int(h * (1.0 - ROI_BOT))
        roi    = frame[y0:y1, :]

        label, overlay, r_px, y_px = self._detect(roi)

        self.get_logger().info(
            f'[PERCEPTION] at ({ox:.1f},{oy:.1f}) — red:{r_px} yellow:{y_px} → {label}')

        # Always save the snapshot regardless of result (useful for debugging)
        ts    = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:19]
        fname = f'{label}_{ts}.jpg'
        fpath = os.path.join(SAVE_DIR, fname)

        # Paste ROI overlay back into full frame for context
        annotated         = frame.copy()
        annotated[y0:y1] = overlay
        cv2.rectangle(annotated, (0, y0), (w, y1), (255, 255, 0), 1)
        cv2.imwrite(fpath, annotated)

        if label == 'none':
            return

        if self._is_duplicate(label, ox, oy):
            self.get_logger().info(f'[PERCEPTION] dedup — {label} already logged near ({ox:.1f},{oy:.1f})')
            return

        entry = {'label': label, 'x': round(ox, 2), 'y': round(oy, 2),
                 'image': fpath, 'timestamp': ts}
        self._session.append(entry)
        self._append_log(entry)

        out      = String()
        out.data = json.dumps(entry)
        self._det_pub.publish(out)
        self.get_logger().info(f'[PERCEPTION] LOGGED: {label} at ({ox:.1f},{oy:.1f}) — {fname}')

    # ── detection ─────────────────────────────────────────────────────────────

    def _detect(self, roi):
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        red_mask = cv2.bitwise_or(
            cv2.inRange(hsv, RED_LO1, RED_HI1),
            cv2.inRange(hsv, RED_LO2, RED_HI2))
        yel_mask = cv2.inRange(hsv, YEL_LO, YEL_HI)

        r_px = int(cv2.countNonZero(red_mask))
        y_px = int(cv2.countNonZero(yel_mask))

        overlay = roi.copy()

        # Draw bounding boxes for all significant contours
        for mask, colour in [(red_mask, (0, 0, 255)), (yel_mask, (0, 255, 255))]:
            for cnt in cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]:
                if cv2.contourArea(cnt) > 150:
                    x, y, bw, bh = cv2.boundingRect(cnt)
                    cv2.rectangle(overlay, (x, y), (x + bw, y + bh), colour, 2)

        if r_px >= MIN_PIXELS and r_px >= y_px:
            label   = 'red'
            txt_col = (0, 0, 255)
        elif y_px >= MIN_PIXELS and y_px > r_px:
            label   = 'yellow'
            txt_col = (0, 200, 255)
        else:
            label   = 'none'
            txt_col = (180, 180, 180)

        cv2.putText(overlay, f'{label}  R:{r_px} Y:{y_px}',
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.8, txt_col, 2)
        return label, overlay, r_px, y_px

    # ── helpers ───────────────────────────────────────────────────────────────

    def _is_duplicate(self, label: str, x: float, y: float) -> bool:
        for e in self._session:
            if e['label'] == label and math.hypot(x - e['x'], y - e['y']) < DEDUP_DIST:
                return True
        return False

    def _append_log(self, entry: dict):
        log = []
        if os.path.exists(self._log_path):
            try:
                with open(self._log_path) as f:
                    log = json.load(f)
            except Exception:
                pass
        log.append(entry)
        with open(self._log_path, 'w') as f:
            json.dump(log, f, indent=2)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
