import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class ModePublisherNode(Node):

    def __init__(self):
        super().__init__('mode_publisher_node')

        self.subscription = self.create_subscription(Joy, '/joy', self.joy_callback, 10)
        self.publisher = self.create_publisher(String, '/mode', 10)

        self.mode = "MANUAL"

    def joy_callback(self, msg):
        x_button = msg.buttons[0]
        o_button = msg.buttons[1]

        if x_button:
            self.mode = "AUTO"
            self.get_logger().info("X pressed, AUTO_MODE ON")
        if o_button:
            self.mode = "MANUAL"
            self.get_logger().info("O pressed, MANUAL_MODE ON")
        self.get_logger().info(f"axes: {msg.axes}")
        self.get_logger().info(f"buttons: {msg.buttons}")

        msg_out = String()
        msg_out.data = self.mode
        self.publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = ModePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()