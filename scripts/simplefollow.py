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
        rospy.init_node("slam")

    def drive_control(self, msg):
        #Main function
        drive_command = AckermannDriveStamped(self.header, AckermannDrive(steering_angle=new_steering_angle, speed=2.0))
        self.drive_pub.publish(drive_command)

if __name__ == "__main__":

    node = ObjectDetectorNode()

    rospy.spin()
