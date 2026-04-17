colcon build
source install/setup.bash

ros2 run joy joy_node --ros-args -p device:=/dev/input/js0 &
ros2 run teleop_twist_joy teleop_node --ros-args -p require_enable_button:=false -r /cmd_vel:=cmd_vel_teleop &
ros2 run ariaNode ariaNode -rp /dev/ttyUSB0 &
ros2 run mode_publisher_package mode_publisher_node &
ros2 launch sick_scan_xd sick_tim_5xx. launch.py &

wait