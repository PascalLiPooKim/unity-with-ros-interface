#!/usr/bin/env python2

import sys
import numpy as np
import rospy
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Int64

class CameraStream():
    def __init__(self):
        rospy.init_node("test")
        self.imageSubscriber = rospy.Subscriber("/iris/usb_cam/image_raw", Image, self.imageCallback)
        self.bridge = CvBridge()
        self.gst_str = "appsrc ! videoconvert ! video/x-raw, format=I420, width=320, height=240, framerate=24/1 ! fdsink"
        self.out = cv2.VideoWriter(self.gst_str, 0, 24, (320, 240), True)
        self.img = np.zeros((240, 320, 3), np.uint8)

    def imageCallback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg)

    def main(self):
        while not rospy.is_shutdown():
            self.out.write(self.img)

if __name__ == "__main__":
    cs = CameraStream()
    cs.main()
