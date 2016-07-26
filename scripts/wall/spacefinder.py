#!/usr/bin/python

import rospy
import numpy as np
from ackermann_msgs.msg import *
from sensor_msgs.msg import *
from std_msgs.msg import *

class spacefinder:
    def __init__(self):
        rospy.init_node("slam")
        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped)
        self.vision = rospy.Subscriber("/scan", LaserScan, self.drive_control)
        self.kp = 0.0027

    def drive_control(self, msg):
        #Main function
        error = (540 - self.findLongest(msg.ranges)) * -1
        angle = error * self.kp

        drive_command = AckermannDriveStamped()
        drive_command.drive.speed = 1.0
        drive_command.drive.steering_angle = angle
        self.drive_pub.publish(drive_command)

    def findLongest(self, ranges):
        rangenum = 39
        slices = []
        for i in range(180, 901):
            summ = 0
            for x in range(0, (rangenum+1)/2):
                if x != 0:
                    summ += ranges[i+x]
                summ += ranges[i-x]
            summ /= rangenum
            slices.append(summ)
        max = 0
        for i in range(0, len(slices)):
            if(slices[i] > slices[max]):
                max = i
        return max + 180

if __name__ == "__main__":
    node = spacefinder()
    rospy.spin()
