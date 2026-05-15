import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
import tf_transformations
import math
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

def YawFromQuaternion(x, y, z, w):
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if (norm == 0):
        return 0
    yaw = math.atan2(2.0 * (x*y + z*w), 1.0 - 2.0 * (y*y + z*z))
    return yaw

class OdomNode(Node):

    def __init__(self):
        super().__init__('odom_node')

        self.sub = self.create_subscription(Pose2D,'/robot_pose',self.cb,10)
        self.pub = self.create_publisher(Odometry, '/odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    def cb(self, msg):
        odom = Odometry()

        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        odom.pose.pose.position.x = msg.x
        odom.pose.pose.position.y = msg.y
        odom.pose.pose.position.z = 0.0

        q = tf_transformations.quaternion_from_euler(0, 0, msg.theta)
        odom.pose.pose.orientation = Quaternion(
            x=q[0], y=q[1], z=q[2], w=q[3]
        )

        self.pub.publish(odom)
        """
        # Position & Orientation
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = YawFromQuaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        #self.get_logger().info(f"x: {x:.2f}, y: {y:.2f}, yaw: {yaw:.2f}")
        """
        # tf
        t = TransformStamped()

        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0
        t.transform.rotation = Quaternion(
            x=q[0],
            y=q[1],
            z=q[2],
            w=q[3]
        )
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()