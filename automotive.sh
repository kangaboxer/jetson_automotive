#! /bin/sh
sleep 10
rosrun rosserial_python serial_node.py /dev/arduino _baud:=115200 &
sleep 5
rosrun uvc_camera uvc_camera_node image_raw:=/camera/image_raw &
rosrun hokuyo_node hokuyo_node ~port:=/dev/hokuyo &
rosrun automotive_tf automotive_tf_node &
sleep 3
cd ~/catkin_ws/src/ORB_SLAM/
rosrun ORB_SLAM ORB_SLAM ./Data/ORBvoc.txt ./Data/Settings.yaml
