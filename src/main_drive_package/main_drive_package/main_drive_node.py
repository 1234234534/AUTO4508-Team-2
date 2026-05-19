import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist

class MainDriveNode(Node):

    def __init__(self):
        super().__init__('main_drive_node')

        # Store Current Mode
        self.mode = "MANUAL"
        self.allow = "OFF"

        # Subscribe to mode, teleop_cmd_vel, (and nav2_cmd_vel later too)
        self.subscription = self.create_subscription(String, '/mode', self.mode_callback, 10)
        self.subscription2 = self.create_subscription(Twist, '/cmd_vel_teleop', self.teleop_callback, 10)
        self.subscription3 = self.create_subscription(Twist, '/cmd_vel_final', self.pointandshoot_callback, 10)

        # Publish to cmd_vel
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Store teleop cmd_vel (Later store nav2 cmd_vel too)
        self.teleop_cmd = Twist()
        self.pointandshoot_cmd = Twist()

    def mode_callback(self, msg):
        # Store current settings
        self.mode, self.allow = msg.data.split(":")

        #update output
        self.update_output()

    def teleop_callback(self, msg):
        # Store teleop_twist_joy msg
        self.teleop_cmd = msg

        #Update cmd_vel
        self.update_output()

    def pointandshoot_callback(self, msg):
        self.pointandshoot_cmd = msg
        self.update_output()

    def update_output(self):
        # Twist Message
        twist = Twist()

        if self.mode == "MANUAL" and self.allow == "ON":
            # Activate Teleop_twist_joy
            twist = self.teleop_cmd

        elif self.mode == "AUTO" and self.allow == "ON":
            # Set to 0.2 forward in x direction
            #twist.linear.x = 0.2
            #twist.angular.z = 0.0
            #LATER
            twist = self.pointandshoot_cmd

        else:
            # Set to 0 speed
            twist.linear.x = 0.0
            twist.angular.z = 0.0

        #self.get_logger().info(f"{twist.linear.x}, {twist.angular.z}", throttle_duration_sec=1.0)
        self.publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = MainDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()