colcon build
source install/setup.bash

export ROS_DOMAIN_ID=2

# Controller
ros2 run joy joy_node --ros-args -p device:=/dev/input/js0 &
ros2 run teleop_twist_joy teleop_node \
    --ros-args \
    -p require_enable_button:=false \
    -p axis_linear.x:=1 \
    -p scale_linear.x:=4.0 \
    -p axis_angular.yaw:=0 \
    -p scale_angular.yaw:=1.0 \
    -r /cmd_vel:=cmd_vel_teleop &

# Aria
ros2 run ariaNode ariaNode -rp /dev/ttyUSB0 &

#Devices
ros2 launch depthai_ros_driver camera.launch.py device_id:=19443010C11F481300 &
#ros2 launch sick_scan_xd sick_tim_5xx. launch.py &
#ros2 run nmea_navsat_driver nmea_serial_driver --ros-args -p port:=/dev/ttyACM0 -r fix:=/gps/fix &
ros2 launch phidgets_spatial spatial-launch.py &
#ip addr add 192.168.198.1/24 dev enp89s0 2>/dev/null || true
ros2 launch lakibeam1 lakibeam1_scan.launch.py &

#TF Tree Static Transformations
#ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link &
ros2 run tf2_ros static_transform_publisher 0.2 0 0 0 0 0 base_link oak-d-base-frame &
ros2 run tf2_ros static_transform_publisher 0.2 0 0.104 0 0 0 base_link laser &

# Our Nodes
#ros2 run mode_publisher_package mode_publisher_node &
ros2 run main_drive_package main_drive_node &
ros2 run main_drive_package odom_node &

#ros2 run main_drive_package pointandshoot_node &

#SLAM
ros2 launch slam_toolbox online_async_launch.py slam_params_file:=$(pwd)/src/pioneer_robot/config/slam_params.yaml &
#rviz2
#rviz2 -d rviz2_config.rviz &
ros2 run foxglove_bridge foxglove_bridge &

sleep 5

#Nav2
ros2 launch $(pwd)/src/pioneer_robot/resources/launch/navigation_launch.py \
    params_file:=$(pwd)/src/pioneer_robot/config/nav2_params.yaml \
    use_sim_time:=false &

sleep 5

ros2 run pioneer_robot explorer &
ros2 run pioneer_robot perception &
#ros2 run main_drive_package take_image_node &

# Record Journey
ros2 bag record /cmd_vel /scan /odom /detections -o /rosbags/journey_$(date +%Y%m%d_%H%M%S) &

# E-Stop Snapshots
ros2 bag record --snapshot-mode --max-cache-size 50000000 /cmd_vel /scan /odom /detections -o /rosbags/estop_$(date +%Y%m%d_%H%M%S) &

#GUI
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
wait