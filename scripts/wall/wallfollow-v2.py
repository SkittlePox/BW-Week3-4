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
        #self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.thread_image, queue_size=1)
        self.vision = rospy.Subscriber("/scan", LaserScan, self.drive_control)
        self.joystick = rospy.Subscriber("/vesc/joy", Joy, self.handle_joy)

        # OpenCV Setup
        self.bridge = CvBridge()

        # Other global variables
        self.kpd = 1        # For distance
        self.kpa = 1 / 120.0   # For angle

        self.run = True
        self.direction = 1  # 1 for right, -1 for left
        self.the_time = rospy.Time.now()
        self.joy_time = rospy.Time.now()

    def drive_control(self, msg):
        if(self.run):
            d0 = 0.5   # Optimal distance from wall
            a = 60      # Distance between collection points
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
            print(angle, error)
            self.drive(error, 2)

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
        if(speed != 0):
            print(speed)
            # Ramp speed
        drive_command = AckermannDriveStamped()
        drive_command.drive.speed = speed
        drive_command.drive.steering_angle = angle

        self.drive_pub.publish(drive_command)

    def handle_joy(self, msg):
        if(msg.buttons[0] == 1 and rospy.Time.now() - self.joy_time >= rospy.Duration(0.5, 0)):    # A button
            self.run = not self.run
            self.joy_time = rospy.Time.now()
        if(msg.buttons[2] == 1):    # X button
            self.direction = -1
        elif(msg.buttons[1] == 1):  # B button
            self.direction = 1

    def thread_image(self, msg):
        thread = threading.Thread(target=self.process_image, args=(msg,))
        thread.setDaemon(True)
        thread.start()

    def process_image(self, image_msg):
        # Threading stuff
        if not self.thread_lock.acquire(False):
            return

        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        the_one, color_scheme, display_text = self.color_search(image_cv)

        if(the_one is not None):
            self.paint_image(image_cv, the_one, color_scheme, display_text)

    def paint_image(self, image_cv, the_one, color_scheme, display_text):
        cv2.drawContours(image_cv, [the_one], -1, (color_scheme.b, color_scheme.g, color_scheme.r))
        x, y, w, h = cv2.boundingRect(the_one)
        v2.rectangle(image_cv, (x, y), (x + w, y + h), (color_scheme.b, color_scheme.g, color_scheme.r, 2))
        cv2.circle(image_cv, (x + w / 2, y + h / 2), 4, (255, 255, 255), -1)
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(image_cv, display_text, (x + w / 2, y + 3 * h / 4), font, 2, (255, 255, 255), 2)

        self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))


    def color_search(self, image_cv):
        threshold = 7000
        image_hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

        filters_red = [np.array([0, 165, 70]), np.array([6, 255, 255])]  # Red
        filters_red2 = [np.array([170, 165, 140]), np.array([180, 255, 255])]  # Red
        filters_green = [np.array([50, 100, 30]), np.array([88, 255, 255])]  # Green
        filters_yellow = [np.array([22, 150, 84]), np.array([40, 255, 190])]  # Yellow
        filters_blue = [np.array([106, 127, 40]), np.array([127, 235, 215])]  # Blue
        filters_pink = [np.array([144, 91, 116]), np.array([165, 145, 255])]  # Pink

        mask_red = cv2.inRange(image_hsv, filters_red[0], filters_red[1])
        mask_red2 = cv2.inRange(image_hsv, filters_red2[0], filters_red2[1])
        mask_green = cv2.inRange(image_hsv, filters_green[0], filters_green[1])
        mask_yellow = cv2.inRange(image_hsv, filters_yellow[0], filters_yellow[1])
        mask_blue = cv2.inRange(image_hsv, filters_blue[0], filters_blue[1])
        mask_pink = cv2.inRange(image_hsv, filters_pink[0], filters_pink[1])

        # Bridging two red filters
        mask_red = cv2.bitwise_or(mask_red, mask_red2)

        mask_red = cv2.GaussianBlur(mask_red, (3, 3), 0)
        mask_green = cv2.GaussianBlur(mask_green, (3, 3), 0)
        mask_yellow = cv2.GaussianBlur(mask_yellow, (3, 3), 0)
        mask_blue = cv2.GaussianBlur(mask_blue, (3, 3), 0)
        mask_pink = cv2.GaussianBlur(mask_pink, (3, 3), 0)

        contours_red = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        contours_green = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        contours_yellow = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        contours_blue = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        contours_pink = cv2.findContours(mask_pink, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

        test_red = len(contours_red) > 0
        test_green = len(contours_green) > 0
        test_yellow = len(contours_yellow) > 0
        test_blue = len(contours_blue) > 0
        test_pink = len(contours_pink) > 0

        if not (test_red or test_green or test_yellow or test_blue or test_pink):  # TODO implement clock check with time.clock() - 5 > the_time, omitted for debugging purposes
            return

        contours = [contours_red, contours_green, contours_yellow, contours_blue, contours_pink]
        tests = [test_red, test_green, test_yellow, test_blue, test_pink]
        colors = ["red", "green", "yellow", "blue", "pink"]
        color_objects = [std_msgs.msg.ColorRGBA(255.0, 0.0, 0.0, 0.0), std_msgs.msg.ColorRGBA(0.0, 255.0, 0.0, 0.0), std_msgs.msg.ColorRGBA(255.0, 255.0, 0.0, 0.0), std_msgs.msg.ColorRGBA(0.0, 0.0, 255.0, 0.0), std_msgs.msg.ColorRGBA(255.0, 0.0, 255.0, 0.0)]

        # Populated Later
        largest_areas = []
        largest_contours = []

        # Winning contour specific variables
        the_one = None  # The winning contour
        display_text = ""
        color_scheme = None  # What color box to draw around it

        for i in range(0, len(tests)):
            if(tests[i]):   # If any contours were found of the 'i'th color
                max_contour = contours[i][0]
                for j in range(0, len(contours[i])):
                    if(cv2.contourArea(contours[i][j]) > cv2.contourArea(max_contour)):
                        max_contour = contours[i][j]
                if(i == 4):  # Pink
                    largest_areas.append(cv2.contourArea(max_contour) * 1.5)
                else:
                    largest_areas.append(cv2.contourArea(max_contour))  # Place the largest found contour's area in the largest_areas array
                largest_contours.append(max_contour)                # Place the contour itself in the largest_contours array
            else:   # If no contours exist for a given mask populated it with empty values
                largest_areas.append(0)
                largest_contours.append(np.array([[0, 0]]))

        largest = max(largest_areas)
        for i in range(0, len(largest_areas)):  # Finds the winning contour
            if(largest_areas[i] == largest):    # This is the winning contour TODO implement threshold for size!
                the_one = largest_contours[i]
                display_text = colors[i]
                color_scheme = color_objects[i]
        if (largest < threshold):
            return None, None, None

        return the_one, color_scheme, display_text

if __name__ == "__main__":
    rospy.init_node('Wallfollow')
    main = Wallfollow()
    rospy.spin()
