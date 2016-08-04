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
            '/camera/rgb/image_rect_color', Image, self.cbImage, queue_size=1)
        self.pub = rospy.Publisher('/detect', Int32MultiArray, queue_size=1)

        self.pub_image = rospy.Publisher("~echo_image", Image, queue_size=1)
        self.bridge = CvBridge()

    def cbImage(self, msg):
        thread = threading.Thread(target=self.processImage, args=(msg,))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, image_msg):
        noneFound = False
        if not self.thread_lock.acquire(False):
            return

        ms = findPoly(image_msg, np.array([54, 30, 60]),
                      np.array([72, 255, 255]))

        self.thread_lock.release()

    def findPoly(self, mn, mx):
        img = self.bridge.imgmsg_to_cv2(image_msg)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, mn, mx)
        mask = cv2.erode(mask, (3, 3), iterations=1)
        contours, h = cv2.findContours(mask,
                                       cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, (0, 0, 255), 2)
        sorted_contours = sorted(contours, key=lambda c: cv2.contourArea(c),
                                 reverse=True)
        if len(sorted_contours < 1):
            return None

        c = sorted_contours[0]
        area = cv2.contourArea(c)

        perim = cv2.arcLength(c, True)
        approx = cv2.approxPolyDP(c, 0.05 * perim, True)

        if len(approx) != 4:
            return None

        cv2.drawContours(img, [c], -1, (255, 0, 0), 3)
        cv2.drawContours(img, [approx], -1, (0, 255, 0), 5)

        M = cv2.moments(approx)
        x, y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

        msg = Int32MultiArray()
        msg.data = [x, area, color]  # area or height?
        try:
            self.pub_image.publish(
                self.bridge.cv2_to_imgmsg(img, "bgr8"))
        except CvBridgeError as e:
            print(e)

        return msg

if __name__=="__main__":
    rospy.init_node('Vision')
    colortracker = ColorTracker()
    rospy.spin()
