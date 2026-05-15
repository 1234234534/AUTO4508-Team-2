#!/usr/bin/env python3
"""
Greek Letter Detector Node
- Subscribes to camera feed (/oak/rgb/image_rect)
- Periodically sends frames to Claude Vision API
- Logs whether a Greek letter is detected or not
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import base64
import json
import requests


class GreekLetterDetector(Node):
    def __init__(self):
        super().__init__('greek_letter_detection')

        self.bridge = CvBridge()
        self.last_detection_time = self.get_clock().now()

        # How often to check for letters (seconds)
        self.detection_interval = 3.0

        self.image_sub = self.create_subscription(
            Image,
            '/oak/rgb/image_rect',
            self.image_callback,
            10
        )

        self.get_logger().info('Greek Letter Detector started')

    def image_callback(self, msg):
        """Process incoming camera frames"""
        now = self.get_clock().now()
        elapsed = (now - self.last_detection_time).nanoseconds / 1e9

        if elapsed < self.detection_interval:
            return

        self.last_detection_time = now

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 85])
            image_b64 = base64.standard_b64encode(buffer).decode('utf-8')
            self.detect_greek_letter(image_b64)

        except Exception as e:
            self.get_logger().error(f'Image processing error: {str(e)}')

    def detect_greek_letter(self, image_b64):
        """Send image to Claude API for Greek letter detection"""
        try:
            response = requests.post(
                'https://api.anthropic.com/v1/messages',
                headers={'Content-Type': 'application/json'},
                json={
                    'model': 'claude-sonnet-4-20250514',
                    'max_tokens': 200,
                    'messages': [
                        {
                            'role': 'user',
                            'content': [
                                {
                                    'type': 'image',
                                    'source': {
                                        'type': 'base64',
                                        'media_type': 'image/jpeg',
                                        'data': image_b64
                                    }
                                },
                                {
                                    'type': 'text',
                                    'text': (
                                        'Is there a hand-drawn Greek letter visible in this image? '
                                        'Respond ONLY in this exact JSON format with no other text: '
                                        '{"detected": true, "letter_name": "Alpha"} '
                                        'or {"detected": false}'
                                    )
                                }
                            ]
                        }
                    ]
                },
                timeout=10
            )

            if response.status_code == 200:
                result_text = response.json()['content'][0]['text'].strip()
                result = json.loads(result_text)

                if result.get('detected'):
                    self.get_logger().info(
                        f'LETTER DETECTED: {result.get("letter_name", "Unknown")}'
                    )
                else:
                    self.get_logger().info('No letter detected')
            else:
                self.get_logger().warning(f'API error: {response.status_code}')

        except json.JSONDecodeError:
            self.get_logger().warning('Could not parse API response')
        except Exception as e:
            self.get_logger().error(f'API call failed: {str(e)}')


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