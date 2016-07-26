#!/usr/bin/python

import rospy
import numpy as np
from ackermann_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *

class ObjectDetectorNode:
    def __init__(self):
        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped)
        self.vision = rospy.Subscriber("/scan", LaserScan, self.drive_control)
        self.right = False
        self.target = 0.5
        self.kp = 0.5
        rospy.init_node("slam")

    def drive_control(self, msg):
        #Main function
        if self.right:
            ranges = msg.ranges[90:360]
            flipConstant = -1
        else:
            ranges = msg.ranges[990:720]
            flipConstant = 1

        error = min(ranges) - self.target
        angle = (error * self.kp * flipConstant)

        drive_command = AckermannDriveStamped(std_msgs.msg.Header(), AckermannDrive(steering_angle=angle, speed=1.0))
        self.drive_pub.publish(drive_command)

if __name__ == "__main__":
    node = ObjectDetectorNode()
    rospy.spin()
