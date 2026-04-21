colcon build
source install/setup.bash

# Controller
ros2 run joy joy_node --ros-args -p device:=/dev/input/js0 &
ros2 run teleop_twist_joy teleop_node \
    --ros-args \
    -p require_enable_button:=false \
    -p axis_linear.x:=1 \
    -p scale_linear.x:=1.0 \
    -p axis_angular.yaw:=0 \
    -p scale_angular.yaw:=1.0 \
    -r /cmd_vel:=cmd_vel_teleop &

# Aria
ros2 run ariaNode ariaNode -rp /dev/ttyUSB0 &

#Devices
#ros2 launch depthai_ros_driver camera.launch.py device_id:=19443010C11F481300 &

ros2 launch sick_scan_xd sick_tim_5xx. launch.py &
ros2 run nmea_navsat_driver nmea_serial_driver \
    --ros-args \
    -p port:=/dev/ttyACM0 \
    -r fix:=/gps/fix &

# Our Nodes
ros2 run mode_publisher_package mode_publisher_node &
ros2 run main_drive_package main_drive_node &
ros2 run main_drive_package odom_node &

wait