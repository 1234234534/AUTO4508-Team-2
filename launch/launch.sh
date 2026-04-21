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
ros2 launch phidgets_spatial spatial-launch.py &

# Our Nodes
ros2 run mode_publisher_package mode_publisher_node &
ros2 run main_drive_package main_drive_node &
ros2 run main_drive_package odom_node &

# slam
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time:=false \
    slam_params_file:=src/pioneer_robot/config/slam_params.yaml &

#nav 2
ros2 run nav2_controller controller_server \
    --ros-args --params-file src/pioneer_robot/config/nav2_params.yaml \
    -r cmd_vel:=cmd_vel_nav &
ros2 run nav2_smoother smoother_server \
    --ros-args --params-file src/pioneer_robot/config/nav2_params.yaml &
ros2 run nav2_planner planner_server \
    --ros-args --params-file src/pioneer_robot/config/nav2_params.yaml &
ros2 run nav2_behaviors behavior_server \
    --ros-args --params-file src/pioneer_robot/config/nav2_params.yaml \
    -r cmd_vel:=cmd_vel_nav &
ros2 run nav2_bt_navigator bt_navigator \
    --ros-args --params-file src/pioneer_robot/config/nav2_params.yaml &
ros2 run nav2_waypoint_follower waypoint_follower \
    --ros-args --params-file src/pioneer_robot/config/nav2_params.yaml &
ros2 run nav2_velocity_smoother velocity_smoother \
    --ros-args --params-file src/pioneer_robot/config/nav2_params.yaml \
    -r cmd_vel:=cmd_vel_nav &
ros2 run nav2_collision_monitor collision_monitor \
    --ros-args --params-file src/pioneer_robot/config/nav2_params.yaml &
ros2 run nav2_lifecycle_manager lifecycle_manager \
    --ros-args \
    -p use_sim_time:=false \
    -p autostart:=true \
    -p node_names:='["controller_server","smoother_server","planner_server","behavior_server","bt_navigator","waypoint_follower","velocity_smoother","collision_monitor"]' &

wait