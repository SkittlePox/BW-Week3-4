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
        self.pub = rospy.Publisher('detect', Int32MultiArray, queue_size=1)

        self.pub_image = rospy.Publisher("~echo_image", Image, queue_size=1)
        self.bridge = CvBridge()

    def cbImage(self, msg):
        thread = threading.Thread(target=self.processImage, args=(msg,))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        ms = self.findPoly(image_msg, 0)
        if(ms is None):
            ms = self.findPoly(image_msg, 1)
        if(ms is None):
            ms = self.getDefaultMsg()
        print(ms.data)
        self.pub.publish(ms)
        self.thread_lock.release()

    def getDefaultMsg(self):
        msg = Int32MultiArray()
        msg.data = [0, 0, -1]
        return msg

    def findPoly(self, image_msg, mn):
        img = self.bridge.imgmsg_to_cv2(image_msg)
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        if(mn == 1):
            greens = [np.array([54, 110, 10]), np.array([72, 180, 200])]
            mask = cv2.inRange(hsv, greens[0], greens[1])
        else:
            filters_red = [np.array([0, 125, 70]), np.array([8, 255, 255])]
            filters_red2 = [np.array([170, 100, 70]),
                            np.array([180, 255, 255])]
            mask_red = cv2.inRange(hsv, filters_red[0], filters_red[1])
            mask_red2 = cv2.inRange(hsv, filters_red2[0],
                                    filters_red2[1])
            mask = cv2.bitwise_or(mask_red, mask_red2)

        mask = cv2.erode(mask, (3, 3), iterations=1)
        contours, h = cv2.findContours(mask,
                                       cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(img, contours, -1, (0, 0, 255), 2)
        sorted_contours = sorted(contours, key=lambda c: cv2.contourArea(c),
                                 reverse=True)
        self.pub_image.publish(
                self.bridge.cv2_to_imgmsg(img, "bgr8"))
        if len(sorted_contours) < 1:
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
        if(M['m00'] == 0 or M['m01'] == 0):
            return None
        x, y = int(M['m10']/M['m00']), int(M['m01']/M['m00'])

        msg = Int32MultiArray()
        msg.data = [x, area, mn]
        return msg

if __name__=="__main__":
    rospy.init_node('Vision')
    vision = Vision()
    rospy.spin()
