#!/usr/bin/python

import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class WallFollower:
    def __init__(self):
        self.drive_pub = rospy.Publisher(
            "/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped)
        rospy.Subscriber("/scan", LaserScan, self.scan_received)

    def scan_received(self, msg):
        self.drive(self.calculateAngle(msg), 1)

    def calculateAngle(self, msg):
        
        return 0

    def drive(self, angle, speed):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.drive_pub.publish(msg)

if __name__ == "__main__":
    node = WallFollower()
    rospy.init_node("wall_follow")
    rospy.spin()
