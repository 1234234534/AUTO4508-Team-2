import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy, Image
from cv_bridge import CvBridge
import cv2


class JoyImageCapture(Node):
    def __init__(self):
        super().__init__('joy_image_capture')

        self.bridge = CvBridge()

        self.latest_image = None
        self.last_button_state = 0

        self.image_sub = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            10
        )

        self.joy_sub = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.get_logger().info("Ready: press TRIANGLE to capture image")

    def image_callback(self, msg):
        self.latest_image = msg

    def joy_callback(self, msg):
        if len(msg.buttons) < 3:
            return

        triangle_pressed = msg.buttons[3]  # PS4 triangle

        # Trigger only on rising edge (0 -> 1)
        if triangle_pressed == 1 and self.last_button_state == 0:
            self.get_logger().info("Triangle Button Pressed")
            self.save_image()

        self.last_button_state = triangle_pressed

    def save_image(self):
        if self.latest_image is None:
            self.get_logger().warn("No image received yet")
            return

        img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')

        filename = f"oak_capture_{self.get_clock().now().to_msg().sec}.png"
        cv2.imwrite(filename, img)

        self.get_logger().info(f"Saved {filename}")


def main():
    rclpy.init()
    node = JoyImageCapture()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()