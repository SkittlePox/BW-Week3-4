#!/usr/bin/python

import cv2
import rospy
import numpy as np
from ackermann_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import threading
import sys
import math
from std_msgs.msg import Float32MultiArray


class Wallfollow:

    def __init__(self):
        self.node_name = "Wallfollow"
        self.thread_lock = threading.Lock()

        # Publishers and Subscribers
        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped, queue_size=1)
        self.pub_image = rospy.Publisher("/echo_image", Image, queue_size=1)
        #self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.process_image, queue_size=1)
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

    def drive_control(self, msg):
        if(self.run):
            d0 = 0.5   # Optimal distance from wall
            a = 40      # Distance between collection points
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
            self.drive(error, 1)

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
        if(msg.buttons[0] == 1):    # A button
            self.run = True
        if(msg.buttons[2] == 1):    # X button
            self.direction = -1
        elif(msg.buttons[1] == 1):  # B button
            self.direction = 1

if __name__ == "__main__":
    rospy.init_node('Wallfollow')
    main = Wallfollow()
    rospy.spin()
