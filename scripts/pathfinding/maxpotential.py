#!/usr/bin/python

import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class MaxPotential:
    def __init__(self):
        rospy.init_node("max_potential")
        self.drive_pub = rospy.Publisher(
            "/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped)
        rospy.Subscriber("/scan", LaserScan, self.scan_received)

    def scan_received(self, msg):
        print(self.calculateCharges(msg))
        # self.drive(self.calculateAngle(msg), self.calculateSpeed(msg))

    # def calculateSpeed(self, msg):

    # def calculateAngle(self, msg):


    def calculateCharges(self, msg):
        return [1/(x**2) for x in msg.ranges]

    def drive(self, angle, speed):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.drive_pub.publish(msg)

if __name__ == "__main__":
    node = MaxPotential()
    rospy.spin()
