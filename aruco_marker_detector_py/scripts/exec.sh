#!/usr/bin/bash

sudo chmod 666 /dev/$(ls /dev | grep video | awk 'NR%2==1' | tail -n 1)
ros2 run v4l2_camera v4l2_camera_node --ros-args -p video_device:=/dev/$(ls /dev | grep video | awk 'NR%2==1' | tail -n 1)
