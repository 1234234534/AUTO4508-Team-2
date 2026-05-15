#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import pytesseract
import numpy as np


class GreekLetterDetection(Node):
    def __init__(self):
        super().__init__('greek_letter_detection')
        self.bridge = CvBridge()
        self.last_detection_time = self.get_clock().now()
        self.detection_interval = 3.0

        # All Greek letters to check against
        self.greek_letters = [
            'Α','Β','Γ','Δ','Ε','Ζ','Η','Θ','Ι','Κ','Λ','Μ',
            'Ν','Ξ','Ο','Π','Ρ','Σ','Τ','Υ','Φ','Χ','Ψ','Ω',
            'α','β','γ','δ','ε','ζ','η','θ','ι','κ','λ','μ',
            'ν','ξ','ο','π','ρ','σ','τ','υ','φ','χ','ψ','ω'
        ]

        self.greek_names = {
            'Α': 'Alpha', 'Β': 'Beta', 'Γ': 'Gamma', 'Δ': 'Delta',
            'Ε': 'Epsilon', 'Ζ': 'Zeta', 'Η': 'Eta', 'Θ': 'Theta',
            'Ι': 'Iota', 'Κ': 'Kappa', 'Λ': 'Lambda', 'Μ': 'Mu',
            'Ν': 'Nu', 'Ξ': 'Xi', 'Ο': 'Omicron', 'Π': 'Pi',
            'Ρ': 'Rho', 'Σ': 'Sigma', 'Τ': 'Tau', 'Υ': 'Upsilon',
            'Φ': 'Phi', 'Χ': 'Chi', 'Ψ': 'Psi', 'Ω': 'Omega',
            'α': 'Alpha', 'β': 'Beta', 'γ': 'Gamma', 'δ': 'Delta',
            'ε': 'Epsilon', 'ζ': 'Zeta', 'η': 'Eta', 'θ': 'Theta',
            'ι': 'Iota', 'κ': 'Kappa', 'λ': 'Lambda', 'μ': 'Mu',
            'ν': 'Nu', 'ξ': 'Xi', 'ο': 'Omicron', 'π': 'Pi',
            'ρ': 'Rho', 'σ': 'Sigma', 'τ': 'Tau', 'υ': 'Upsilon',
            'φ': 'Phi', 'χ': 'Chi', 'ψ': 'Psi', 'ω': 'Omega'
        }

        self.image_sub = self.create_subscription(
            Image,
            '/oak/rgb/image_rect',
            self.image_callback,
            10
        )
        self.get_logger().info('Greek Letter Detector started (Tesseract mode)')

    def preprocess_image(self, cv_image):
        """Preprocess image to improve OCR accuracy"""
        # Convert to greyscale
        grey = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Resize to make letter larger (helps OCR)
        scale = 2.0
        grey = cv2.resize(grey, None, fx=scale, fy=scale,
                         interpolation=cv2.INTER_CUBIC)

        # Apply threshold to get clean black/white image
        _, thresh = cv2.threshold(
            grey, 0, 255,
            cv2.THRESH_BINARY + cv2.THRESH_OTSU
        )

        # Denoise
        thresh = cv2.medianBlur(thresh, 3)

        return thresh

    def image_callback(self, msg):
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
        """Use Tesseract OCR to detect Greek letters"""
        try:
            # Run OCR with Greek language
            text = pytesseract.image_to_string(
                processed_image,
                lang='ell',  # Greek language
                config='--psm 10 --oem 3'  # PSM 10 = single character
            ).strip()

            # Check if any detected character is a Greek letter
            detected = None
            for char in text:
                if char in self.greek_letters:
                    detected = char
                    break

            if detected:
                name = self.greek_names.get(detected, 'Unknown')
                self.get_logger().info(f'LETTER DETECTED: {name} ({detected})')
            else:
                self.get_logger().info('No letter detected')

        except Exception as e:
            self.get_logger().error(f'OCR error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = GreekLetterDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Greek Letter Detector')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()