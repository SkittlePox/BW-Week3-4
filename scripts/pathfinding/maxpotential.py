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
        charges = [(q/(((x + 0.01)**2)), math.radians(i/4-135))
                   for i, x in enumerate(msg.ranges)]
        coords = [(x[0] * np.cos(x[1]), x[0] * np.sin(x[1])) for x in charges]
        print(coords[700][0])
        xs = reduce(lambda prev, coord: prev + coord[0], coords)
        ys = reduce(lambda prev, coord: prev + coord[1], coords)
        xs = -xs[0]
        ys = ys[0]
        p_speed = 0.0002
        p_angle = 1
        boost = 1

        speed = (p_speed * math.sqrt(xs**2 + ys**2) * np.sign(xs)) + boost
        angle = p_angle * math.atan2(ys, xs) * np.sign(xs)
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
