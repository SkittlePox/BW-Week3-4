#!/usr/bin/python

import rospy
import math
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class WallFollower:
    def __init__(self):
        rospy.init_node("wall_follow")
        self.drive_pub = rospy.Publisher(
            "/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped)
        rospy.Subscriber("/scan", LaserScan, self.scan_received)
        self.distance_desired = 0.5
        self.Kp = -0.6
        self.Kd = -0.7
        self.left = 880
        self.right = 170
        self.distance_last = 0.5

    def scan_received(self, msg):
        self.drive(self.calculateAngle(msg), 1)

    def calculateAngle(self, msg):
        distance = self.getDistance(msg)
        error = self.distance_desired - distance
        p = self.Kp * error
        d = self.Kd * (distance - self.distance_last)
        angle = p + d

        self.distance_last = distance
        return angle

    def getDistance(self, msg):
        c = self.left
        return min(msg.ranges[c:c+30])

    def drive(self, angle, speed):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.drive_pub.publish(msg)

if __name__ == "__main__":
    node = WallFollower()
    rospy.spin()
