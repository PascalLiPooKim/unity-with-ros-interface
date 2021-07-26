#!/bin/bash

rosrun ros_camera image_view.py | ~/catkin_ws/src/ros_camera/src/rtsp_server "fdsrc fd=0 ! queue2 ! videoparse format=i420 height=480 width=640 framerate=15/1 ! videoconvert ! x264enc bitrate=4096 ! rtph264pay name=pay0 pt=96"
