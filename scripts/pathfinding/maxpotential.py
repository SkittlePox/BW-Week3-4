#!/usr/bin/python

import rospy
import math
import numpy as np
from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class MaxPotential:
    def __init__(self):
        rospy.init_node("max_potential")
        self.drive_pub = rospy.Publisher(
            "/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped)
        rospy.Subscriber("/scan", LaserScan, self.scan_received)

    def scan_received(self, msg):
        q = 0.1
        charges = [(q/(x**2), math.radians((i-180)/4)) for x, i in msg.ranges]
        coords = [(x[0] * np.cos(x[1]), x[0] * np.sin(x[1])) for x in charges]
        xs = reduce(lambda prev, coord: prev + coord[0], coords)
        ys = reduce(lambda prev, coord: prev + coord[1], coords)

        p_speed = 0.2
        p_angle = 1

        speed = p_speed * math.sqrt(xs**2 + ys**2) * np.sign(x)
        angle = p_angle * math.atan2(y, x) * np.sign(x)
        print(angle, speed)
        self.drive(angle, speed)

    def drive(self, angle, speed):
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = angle
        self.drive_pub.publish(msg)

if __name__ == "__main__":
    node = MaxPotential()
    rospy.spin()
