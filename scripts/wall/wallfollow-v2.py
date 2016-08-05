#!/usr/bin/python

import cv2
import rospy
import threading
import sys
import math
import numpy as np
from ackermann_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import *
from geometry_msgs.msg import Point


class Wallfollow:

    def __init__(self):
        self.node_name = "Wallfollow"
        self.thread_lock = threading.Lock()

        # Publishers and Subscribers
        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.pub_image = rospy.Publisher("/echo_image", Image, queue_size=1)
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.thread_image, queue_size=1)
        self.vision = rospy.Subscriber("/scan", LaserScan, self.drive_control)
        self.joystick = rospy.Subscriber("/vesc/joy", Joy, self.handle_joy)

        # OpenCV Setup
        self.bridge = CvBridge()

        # Other global variables
        self.kpd = 0.4        # For distance
        self.kpa = 1 / 150.0   # For angle

        self.run = False
        self.direction = None  # 1 for right, -1 for left
        self.image_processing = False

        self.the_time = rospy.Time.now()
        self.joy_time_drive = rospy.Time.now()
        self.joy_time_image = rospy.Time.now()
        self.shortcut_time = rospy.Time.now()
        self.dirstop = rospy.Time.now()

    def drive_control(self, msg):
        if(self.run and self.direction is not None):
            d0 = 0.6   # Optimal distance from wall
            a = 80      # Distance between collection points
            tolerance = 1   # Span to anerage distances on either side
            midpoints = [900, 180]
            crude_ranges = [[800, 1000], [80, 280]]

            if(self.direction == 1):
                mp = midpoints[1]
                rang = crude_ranges[1]
            else:
                mp = midpoints[0]
                rang = crude_ranges[0]

            ranges = msg.ranges
            crude_distance = min(ranges[rang[0]:rang[1]])

            b = np.mean(ranges[(mp + (a / 2 * 4 * -1 * self.direction) - tolerance * 4): (mp + (a / 2 * 4 * -1 * self.direction) + tolerance * 4)])   # Takes into account tolerance
            f = np.mean(ranges[(mp + (a / 2 * 4 * self.direction) - tolerance * 4): (mp + (a / 2 * 4 * self.direction) + tolerance * 4)])   # Takes into account tolerance

            angle = self.calculate_angle(b, f, a)

            error = (d0 - crude_distance) * self.kpd * self.direction + angle * self.kpa * self.direction
            #print(angle, error)
            self.drive(error, 3)

    def calculate_angle(self, b, f, A):
        # b is the back lidar value, f is the front, and A is the angle between the values
        # Capital is an angle, lowercase is a side length
        if(b == f):     # Bad things can happen when the values equal each other
            b += 0.00001
        a = math.sqrt(math.fabs(b**2 + f**2 - 2 * b * f * math.cos(math.radians(A))))     # Law of Cosines
        B = math.degrees(math.asin(math.sin(math.radians(A)) * min([b, f]) / a))  # Law of Sines
        angle = 90 - B - A / 2.0
        if (b < f):
            angle *= -1
        return angle

    def drive(self, angle, speed):
        drive_command = AckermannDriveStamped()
        drive_command.drive.speed = speed
        drive_command.drive.steering_angle = angle

        self.drive_pub.publish(drive_command)

    def handle_joy(self, msg):
        if(msg.buttons[0] == 1 and rospy.Time.now() - self.joy_time_drive >= rospy.Duration(0.5, 0)):    # A button
            self.run = not self.run
            self.joy_time_drive = rospy.Time.now()
            print("Switching ctrl", self.run)
        if(msg.buttons[2] == 1):    # X button
            self.direction = -1
        elif(msg.buttons[1] == 1):  # B button
            self.direction = 1
        if(msg.button[3] == 1) and rospy.Time.now() - self.joy_time_image >= rospy.Duration(0.5, 0):     # Y button
            self.image_processing = not self.image_processing
            self.joy_time_image = rospy.Time.now()

    def thread_image(self, msg):
        thread = threading.Thread(target=self.process_image, args=(msg,))
        thread.setDaemon(True)
        thread.start()

    def process_image(self, image_msg):
        # Threading stuff
        if not self.thread_lock.acquire(False):
            return

        image_cv = self.bridge.imgmsg_to_cv2(image_msg)

        shortcut, the_one = self.color_search(image_cv)

        if(shortcut is not None and self.image_processing):
            if(shortcut):
                print("Left")
                self.direction = -1
                color = std_msgs.msg.ColorRGBA(0.0, 255.0, 0.0, 0.0)
            elif(not shortcut):
                print("right")
                self.direction = 1
                color = std_msgs.msg.ColorRGBA(255.0, 0.0, 0.0, 0.0)

        if(the_one is not None):
            #image_cv = self.paint_image_text(image_cv, the_one, color_scheme, display_text)
            image_cv = self.paint_image(image_cv, the_one, color)
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))

        self.thread_lock.release()

    def paint_image_text(self, image_cv, the_one, color_scheme, display_text):
        cv2.drawContours(image_cv, [the_one], -1, (color_scheme.b, color_scheme.g, color_scheme.r))
        x, y, w, h = cv2.boundingRect(the_one)
        cv2.rectangle(image_cv, (x, y), (x + w, y + h), (color_scheme.b, color_scheme.g, color_scheme.r, 2))
        cv2.circle(image_cv, (x + w / 2, y + h / 2), 4, (255, 255, 255), -1)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image_cv, display_text, (x + w / 2, y + 3 * h / 4), font, 2, (255, 255, 255), 2)
        return image_cv

    def paint_image(self, image_cv, the_one, color):
        cv2.drawContours(image_cv, [the_one], -1, (color.b, color.g, color.r))
        return image_cv

    def color_search(self, image_cv):
        threshold = 500
        image_hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

        filters_green = [np.array([54, 110, 10]), np.array([72, 180, 200])] # Green
        filters_red = [np.array([0, 125, 70]), np.array([8, 255, 255])]  # Red1
        filters_red2 = [np.array([170, 100, 70]), np.array([180, 255, 255])]  # Red2

        mask_green = cv2.inRange(image_hsv, filters_green[0], filters_green[1])
        mask_red1 = cv2.inRange(image_hsv, filters_red[0], filters_red[1])
        mask_red2 = cv2.inRange(image_hsv, filters_red2[0], filters_red2[1])
        mask_red = cv2.bitwise_or(mask_red1, mask_red2)

        contours_red = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        contours_green = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

        if(len(contours_red) == 0 and len(contours_green) == 0 or rospy.Time.now() - self.shortcut_time < rospy.Duration(1, 0) or rospy.Time.now() - self.dirstop < rospy.Duration(10, 0)):
            return None, None
        self.shortcut_time = rospy.Time.now()

        if(len(contours_red) == 0):
            max_index = 0
            for i in range(0, len(contours_green)):
                if(cv2.contourArea(contours_green[i]) > cv2.contourArea(contours_green[max_index])):
                    max_index = i
            if(cv2.contourArea(contours_green[max_index]) > threshold):
                return True, contours_green[max_index]
            return None, None
        if(len(contours_green) == 0):
            max_index = 0
            for i in range(0, len(contours_red)):
                if(cv2.contourArea(contours_red[i]) > cv2.contourArea(contours_red[max_index])):
                    max_index = i
            if(cv2.contourArea(contours_red[max_index]) > threshold):
                return False, contours_red[max_index]
            return None, None

        max_index_r = 0
        for i in range(0, len(contours_red)):
            if(cv2.contourArea(contours_red[i]) > cv2.contourArea(contours_red[max_index_r])):
                max_index_r = i
        max_index_g = 0
        for i in range(0, len(contours_green)):
            if(cv2.contourArea(contours_green[i]) > cv2.contourArea(contours_green[max_index_g])):
                max_index_g = i

        the_green = contours_green[max_index_g]
        the_red = contours_red[max_index_r]

        if(cv2.contourArea(the_green) > cv2.contourArea(the_red) and cv2.contourArea(the_green) > threshold):
            return True, the_green
        if(cv2.contourArea(the_green) < cv2.contourArea(the_red) and cv2.contourArea(the_red) > threshold):
            return False, the_red

        return None, None

if __name__ == "__main__":
    rospy.init_node('Wallfollow')
    main = Wallfollow()
    rospy.spin()
