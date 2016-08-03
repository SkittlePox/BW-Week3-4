#!/usr/bin/env python
import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray

from cv_bridge import CvBridge, CvBridgeError
import threading


class Vision:
    def __init__(self):
        self.thread_lock = threading.Lock()
        rospy.Subscriber(
            "/camera/rgb/image_rect_color", Image, self.cbImage, queue_size=1)
        self.pub = rospy.Publisher("/detect", Int32MultiArray, queue_size=1)

        self.bridge = CvBridge()

    def cbImage(self, msg):
        thread = threading.Thread(target=self.processImage, args=(msg,))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

        # TODO detect height and moment of red/green

        msg = Int32MultiArray()
        msg.data = [foo, bar, baz]

        self.thread_lock.release()

if __name__=="__main__":
    rospy.init_node('Vision')
    colortracker = ColorTracker()
    rospy.spin()
