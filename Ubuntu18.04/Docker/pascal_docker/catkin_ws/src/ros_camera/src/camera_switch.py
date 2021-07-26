#!/usr/bin/env python2

import sys
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int64

class CameraSwitch():
    def __init__(self):
        rospy.init_node("test")
        self.imageSubscriber = rospy.Subscriber("/image_test1", CompressedImage, self.imageCallback1)
        self.imageSubscriber = rospy.Subscriber("/image_test2", CompressedImage, self.imageCallback2)
        self.intSubscriber = rospy.Subscriber("/channel", Int64, self.channelCallback)
        self.channel = 0
        self.bridge = CvBridge()
        self.gst_str = "appsrc ! videoconvert ! video/x-raw, format=I420, width=640, height=480, framerate=30/1 ! fdsink"
        self.out = cv2.VideoWriter(self.gst_str, 0, 30, (640, 480), True)
        self.img = np.zeros((480, 640, 3), np.uint8)

    def channelCallback(self, msg):
        self.channel = msg.data

    def imageCallback1(self, msg):
        if self.channel == 1:
            self.img = self.bridge.compressed_imgmsg_to_cv2(msg)

    def imageCallback2(self, msg):
        if self.channel == 2:
            self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        
    def main(self):
        while not rospy.is_shutdown():
            self.out.write(self.img)

if __name__ == "__main__":
    print("Init Node")
    cs = CameraSwitch()
    cs.main()
