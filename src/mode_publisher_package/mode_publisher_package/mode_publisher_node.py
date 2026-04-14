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
        self.allowAuto = "OFF"

    def joy_callback(self, msg):
        # Read Relevant Controller States
        x_button = msg.buttons[0]
        o_button = msg.buttons[1]
        trigger = msg.axes[5]

        # Set Mode
        if x_button:
            self.mode = "AUTO"
        if o_button:
            self.mode = "MANUAL"
        if trigger < 0:
            self.allowAuto = "ON"
        elif trigger >= 0:
            self.allowAuto = "OFF"

        #Log Mode
        self.get_logger().info(f"{self.mode}:{self.allowAuto}:{trigger}")

        # Publish Mode
        msg_out = String()
        msg_out.data = f"{self.mode}:{self.allowAuto}"
        self.publisher.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = ModePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()