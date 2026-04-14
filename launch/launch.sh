colcon build
source install/setup.bash

ros2 run joy joy_node --ros-args -p device:=/dev/input/js0 &
ros2 run teleop_twist_joy teleop_node --ros-args -p enable_button:=0 &
ros2 run ariaNode ariaNode -rp /dev/ttyUSB0 &

wait