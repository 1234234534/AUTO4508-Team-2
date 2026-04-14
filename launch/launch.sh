colcon build
source install/setup.bash

ros2 run joy joy_node &
ros2 run teleop_twist_joy teleop_node &
ros2 run ariaNode ariaNode