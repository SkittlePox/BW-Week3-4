#!/usr/bin/python

import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class BugNav:
    def __init__(self):
        rospy.init_node("bug_nav")
        self.drive_pub = rospy.Publisher(
            "/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped)
        rospy.Subscriber("/scan", LaserScan, self.scan_received)
        self.speed = 1.0
        self.fore_thres = 0.5
        self.side_thres = 0.3

        self.center = 540
        self.left = 180
        self.right = 900
        self.bound_45 = 180

    def scan_received(self, msg):
        self.drive(self.calculateAngle(msg), self.calculateSpeed(msg))

    def calculateSpeed(self, msg):
        if(self.getMin(self.left, msg) < self.side_thres and
           self.getMin(self.right, msg) < self.side_thres and
           self.getMin(self.center, msg) < self.fore_thres):
            print("ey")
            return 0.0
        else:
            return self.speed

    def calculateAngle(self, msg):
        if(self.getMin(self.center, msg) < self.fore_thres):
            if(self.getMin(self.left, msg) > self.side_thres):
                return 0.5
            elif(self.getMin(self.right, msg) > self.side_thres):
                return -0.5
        else:
            return 0.0

    def getMin(self, bound, msg):
        return min(msg.ranges[bound-self.bound_45:bound+self.bound_45])

    def drive(self, angle, speed):
        if(angle is None):
            print("wtf")
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.drive_pub.publish(msg)

if __name__ == "__main__":
    node = BugNav()
    rospy.spin()
