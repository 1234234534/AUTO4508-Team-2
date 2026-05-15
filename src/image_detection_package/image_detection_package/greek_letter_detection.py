#!/usr/bin/env python3
"""
Greek Letter Detector Node (EasyOCR)
- Subscribes to camera feed (/oak/rgb/image_rect)
- Uses EasyOCR with Greek language to detect hand-drawn Greek letters
- Logs whether a Greek letter is detected or not
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import easyocr


class GreekLetterDetector(Node):
    def __init__(self):
        super().__init__('greek_letter_detection')

        self.bridge = CvBridge()
        self.last_detection_time = self.get_clock().now()
        self.detection_interval = 3.0

        # Map detected text to Greek letter names
        self.greek_names = {
            'Α': 'Alpha',   'α': 'Alpha',
            'Β': 'Beta',    'β': 'Beta',
            'Γ': 'Gamma',   'γ': 'Gamma',
            'Δ': 'Delta',   'δ': 'Delta',
            'Ε': 'Epsilon', 'ε': 'Epsilon',
            'Ζ': 'Zeta',    'ζ': 'Zeta',
            'Η': 'Eta',     'η': 'Eta',
            'Θ': 'Theta',   'θ': 'Theta',
            'Ι': 'Iota',    'ι': 'Iota',
            'Κ': 'Kappa',   'κ': 'Kappa',
            'Λ': 'Lambda',  'λ': 'Lambda',
            'Μ': 'Mu',      'μ': 'Mu',
            'Ν': 'Nu',      'ν': 'Nu',
            'Ξ': 'Xi',      'ξ': 'Xi',
            'Ο': 'Omicron', 'ο': 'Omicron',
            'Π': 'Pi',      'π': 'Pi',
            'Ρ': 'Rho',     'ρ': 'Rho',
            'Σ': 'Sigma',   'σ': 'Sigma',
            'Τ': 'Tau',     'τ': 'Tau',
            'Υ': 'Upsilon', 'υ': 'Upsilon',
            'Φ': 'Phi',     'φ': 'Phi',
            'Χ': 'Chi',     'χ': 'Chi',
            'Ψ': 'Psi',     'ψ': 'Psi',
            'Ω': 'Omega',   'ω': 'Omega',
        }

        # Initialise EasyOCR with Greek and English
        # gpu=False since robot may not have CUDA
        self.get_logger().info('Loading EasyOCR model (this may take a moment)...')
        self.reader = easyocr.Reader(['el', 'en'], gpu=False)
        self.get_logger().info('EasyOCR model loaded')

        self.image_sub = self.create_subscription(
            Image,
            '/oak/rgb/image_rect',
            self.image_callback,
            10
        )

        self.get_logger().info('Greek Letter Detector started (EasyOCR mode)')

    def preprocess_image(self, cv_image):
        """Preprocess image to improve OCR accuracy"""
        # Convert to greyscale
        grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Upscale to make letters larger and easier to detect
        grey = cv2.resize(grey, None, fx=2.0, fy=2.0,
                          interpolation=cv2.INTER_CUBIC)

        # Apply Otsu threshold for clean black/white image
        _, thresh = cv2.threshold(
            grey, 0, 255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        # Denoise
        thresh = cv2.medianBlur(thresh, 3)

        return thresh

    def image_callback(self, msg):
        """Process incoming camera frames"""
        now = self.get_clock().now()
        elapsed = (now - self.last_detection_time).nanoseconds / 1e9

        if elapsed < self.detection_interval:
            return

        self.last_detection_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            processed = self.preprocess_image(cv_image)
            self.detect_greek_letter(processed)

        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')

    def detect_greek_letter(self, processed_image):
        """Use EasyOCR to detect Greek letters in the image"""
        try:
            # Run OCR - detail=0 returns just text, detail=1 returns bounding boxes too
            results = self.reader.readtext(processed_image, detail=1)

            detected_letter = None
            best_confidence = 0.0

            for (bbox, text, confidence) in results:
                self.get_logger().debug(
                    f'OCR result: "{text}" confidence: {confidence:.2f}'
                )

                # Check each character in the detected text
                for char in text:
                    if char in self.greek_names and confidence > best_confidence:
                        detected_letter = char
                        best_confidence = confidence

            if detected_letter:
                name = self.greek_names[detected_letter]
                self.get_logger().info(
                    f'LETTER DETECTED: {name} ({detected_letter}) '
                    f'[confidence: {best_confidence:.2f}]'
                )
            else:
                self.get_logger().info('No letter detected')

        except Exception as e:
            self.get_logger().error(f'OCR error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = GreekLetterDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Greek Letter Detector')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()