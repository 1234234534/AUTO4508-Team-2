import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations

class OdomNode(Node):

    def __init__(self):
        super().__init__('odom_node')

        self.sub = self.create_subscription(Pose2D,'/robot_pose',self.cb,10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)

    def cb(self, msg):
        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y

        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        odom.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )

        self.pub.publish(odom)

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()