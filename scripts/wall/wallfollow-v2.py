#!/usr/bin/python

import cv2
import rospy
from ackermann_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
import threading
import sys, math
from std_msgs.msg import Float32MultiArray

class Wallfollow:

    def __init__(self):
        self.node_name = "Wallfollow"
        self.thread_lock = threading.Lock()

        # Publishers and Subscribers
        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped)
        self.pub_image = rospy.Publisher("/echo_image", Image, queue_size=1)
        #self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.process_image, queue_size=1)
        self.vision = rospy.Subscriber("/scan", LaserScan, self.drive_control)
        self.joystick = rospy.Subscriber("/vesc/joy", Joy, self.handle_joy)

        # OpenCV Setup
        self.bridge = CvBridge()

        # Other global variables
        self.kp = 1

        self.run = False
        self.direction = 1   #1 for right, -1 for left
        self.the_time = rospy.Time.now()

    def drive_control(self, msg):
        if(self.run):
            ranges = msg.ranges
            crude_distance = min(ranges)
            d0 = 0.35   # Optimal distance from wall

            error = (d0 - crude_distance) * self.kp * self.direction
            self.drive(error, 0)

    def drive(self, angle, speed):
        drive_command = AckermannDriveStamped()
        drive_command.drive.speed = speed
        drive_command.drive.steering_angle = angle

        self.drive_pub.publish(drive_command)

    def handle_joy(self, msg):
        if(msg.buttons[0] == 1):
            self.run = True

if __name__ == "__main__":
    rospy.init_node('Wallfollow')
    main = Wallfollow()
    rospy.spin()
