#!/bin/bash

# Funkcija koja ubija sve pokrenute procese kada pritisneš Ctrl+C
trap "kill 0" EXIT

echo "---  Static Transform Publisher : camera_link -> end effector ---"
rosrun tf2_ros static_transform_publisher -0.05236895 0.03666473 -0.07956362 -0.008328826747513679 0.0027495380545793395 0.71525560603233 -0.6988079054538671 fr3_EE camera_link_proxy &
sleep 1

echo "---  MoveGroup (MoveIt) ---"
ROS_NAMESPACE=robot roslaunch rbkairos_franka_moveit_config move_group.launch &
sleep 12

echo "---  Move MoveIt FR3 ---"
ROS_NAMESPACE=robot python3 move_moveit_fr3.py &
sleep 4

echo "--- Move RB-Kairos ---"
ROS_NAMESPACE=robot python3 move_rbkairos.py &
sleep 1

echo "--- Action Manager ---"
ROS_NAMESPACE=robot python3 action_manager.py &
sleep 2
echo "---Realsense Camera Node ---"
roslaunch realsense2_camera rs_camera.launch align_depth:=true color_width:=640 color_height:=480 color_fps:=15 depth_fps:=15 &
# Drži skriptu aktivnom dok ne pritisneš Ctrl+C
wait
