#!/usr/bin/env python2

import sys
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int64

class CameraSwitch():
    def __init__(self):
        rospy.init_node("DepthView")
        self.imageSubscriber = rospy.Subscriber("/unity/image/compressedDepth", CompressedImage, self.imageCallback)
        self.bridge = CvBridge()
        #self.gst_str = "appsrc ! videoconvert ! video/x-raw, format=I420, width=640, height=480, framerate=30/1 ! fdsink"
        #self.out = cv2.VideoWriter(self.gst_str, 0, 30, (640, 480), True)
        self.img = np.zeros((480, 640, 3), np.uint8)

    def imageCallback(self, msg):
        self.img = self.bridge.compressed_imgmsg_to_cv2(msg)
        
    def main(self):
        while not rospy.is_shutdown():
            cv2.imshow("depth", self.img)
            cv2.waitKey(30)

if __name__ == "__main__":
    print("Init Node")
    cs = CameraSwitch()
    cs.main()