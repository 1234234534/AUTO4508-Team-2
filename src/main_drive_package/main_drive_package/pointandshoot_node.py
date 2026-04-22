import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import Twist
from sensor_msgs.msg import MagneticField


def deg2rad(d):
    return d * math.pi / 180.0


def rad2deg(r):
    return r * 180.0 / math.pi


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000.0  # Earth radius (m)

    phi1 = deg2rad(lat1)
    phi2 = deg2rad(lat2)
    dphi = deg2rad(lat2 - lat1)
    dlambda = deg2rad(lon2 - lon1)

    a = math.sin(dphi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(dlambda/2)**2
    return 2 * R * math.atan2(math.sqrt(a), math.sqrt(1-a))


def bearing(lat1, lon1, lat2, lon2):
    phi1 = deg2rad(lat1)
    phi2 = deg2rad(lat2)
    dlambda = deg2rad(lon2 - lon1)

    y = math.sin(dlambda) * math.cos(phi2)
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(dlambda)

    return (rad2deg(math.atan2(y, x)) + 360) % 360


class PointAndShoot(Node):

    def __init__(self):
        super().__init__('pointandshoot_node')

        # Subscribers
        self.create_subscription(Imu, '/imu/mag', self.imu_callback, 10)
        self.create_subscription(NavSatFix, '/gps/fixed', self.gps_callback, 10)

        # Publisher
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel_pointandshoot', 10)

        # State
        self.current_lat = None
        self.current_lon = None

        self.mag_x = None
        self.mag_y = None
        self.mag_z = None

        # Hardcoded GPS waypoints (lat, lon)
        self.waypoints = [
            (-31.9505, 115.8605),
            (-31.9510, 115.8615),
            (-31.9520, 115.8625),
        ]
        self.current_wp = 0

        # Control loop
        self.timer = self.create_timer(0.2, self.control_loop)

    # ---------------- IMU ----------------
    def imu_callback(self, msg):
        # Adjust if your message differs
        """
        try:
            self.mag_x = msg.mag_field.x
            self.mag_y = msg.mag_field.y
            self.mag_z = msg.mag_field.z
        except:"""
        # fallback (common custom format)
        self.mag_x = msg.magnetic_field.x
        self.mag_y = msg.magnetic_field.y
        self.mag_z = msg.magnetic_field.z
        self.get_logger().info("IMU Callback")

    # ---------------- GPS ----------------
    def gps_callback(self, msg):
        self.current_lat = msg.latitude
        self.current_lon = msg.longitude
        self.get_logger().info("GPS Callback")

    # ---------------- Heading from magnetometer ----------------
    def get_heading(self):
        if self.mag_x is None or self.mag_y is None:
            return None

        # Simple 2D compass heading
        heading = math.atan2(self.mag_y, self.mag_x)
        heading_deg = (rad2deg(heading) + 360) % 360
        return heading_deg

    # ---------------- Main control ----------------
    def control_loop(self):
        if self.current_lat is None or self.current_lon is None:
            self.get_logger().info("Control Loop NONE FOUND")
            return

        if self.current_wp >= len(self.waypoints):
            self.stop_robot()
            self.get_logger().info("Done All Waypoints")
            return

        target_lat, target_lon = self.waypoints[self.current_wp]

        dist = haversine(self.current_lat, self.current_lon, target_lat, target_lon)
        target_bearing = bearing(self.current_lat, self.current_lon, target_lat, target_lon)
        heading = self.get_heading()

        if heading is None:
            self.get_logger().info("No Heading")
            return

        # Error in heading
        error = target_bearing - heading
        error = (error + 180) % 360 - 180  # wrap to [-180, 180]

        cmd = Twist()

        # ---------------- Control law ----------------
        # turn toward target
        cmd.angular.z = 0.01 * error

        # forward speed proportional to alignment
        if abs(error) < 20:
            cmd.linear.x = min(0.5, dist * 0.2)
        else:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)
        self.get_logger().info("Should've just done a cmd_vel_pointandshoot")

        # waypoint reached
        if dist < 1.5:  # meters
            self.get_logger().info(f"Reached waypoint {self.current_wp}")
            self.current_wp += 1
            self.get_logger().info("Reached a Waypoint")

    def stop_robot(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)


def main():
    rclpy.init()
    node = PointAndShoot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()