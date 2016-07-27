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
        #error = (540 - self.findLongest(msg.ranges)) * -1
        error = (540 - self.findMostClear(msg.ranges)) * -1
        angle = error * self.kp

        drive_command = AckermannDriveStamped()
        drive_command.drive.speed = 1.0
        drive_command.drive.steering_angle = angle
        self.drive_pub.publish(drive_command)

    def findLongest(self, ranges):
        bound = 200
        rangenum = 120
        slices = []
        for i in range(bound, 1081-bound):
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

    def findMostClear(self, ranges):
        bound = 200
        rangenum = 120

        slices = []
        for i in range(bound, 1081-bound):
            summ = 0
            for x in range(0, (rangenum+1)/2):
                if x != 0:
                    summ += ranges[i+x]
                summ += ranges[i-x]
            summ /= rangenum
            slices.append(summ)
        indices = range(bound, bound + len(slices))

        for i in range(0, len(slices)):
            maxIndex = i
            for x in range(i, len(slices)):
                if(slices[i] > slices[maxIndex]):
                    maxIndex = i
            slices[i], slices[maxIndex] = slices[maxIndex], slices[i]
            indices[i], indices[maxIndex] = indices[maxIndex], indices[i]   # Flips indices around

        threshold = 0.83
        for i in range(0, len(slices)):
            clarity, certainty = self.isClearPath(ranges, indices[i])
            if(clarity * certainty >= threshold):
                return indices[i]

    def isClearPath(self, ranges, centerIndex):
        left = centerIndex + 360
        right = centerIndex - 360
        left_mid = centerIndex + 240
        right_mid = centerIndex - 240
        left_far = centerIndex + 120
        right_far = centerIndex - 120

        ranges = [0.2, 0.25, 0.4]

        leftPoints = [left, left_mid, left_far]
        rightPoints = [right, right_mid, right_far]

        certaintyAvg = 0    # How certain we are
        clarityAvg = 0    # How clear the path is perceived as

        for l in range(0, len(leftPoints)):
            if(leftPoints[l] < 1080 and leftPoints[l] >= 0):
                certaintyAvg += 1
                print(ranges[leftPoints[l]-4:leftPoints[l]+4])
                print(leftPoints[l])
                if(min(ranges[leftPoints[l]-4:leftPoints[l]+4]) < ranges[l]):
                    clarityAvg += 1

        for r in range(0, len(rightPoints)):
            if(rightPoints[r] < 1080 and rightPoints[r] >= 0):
                certaintyAvg += 1
                if(min(ranges[rightPoints[rpdb]-4:rightPoints[r]+4]) < ranges[r]):
                    clarityAvg += 1

        return clarityAvg/6.0, certaintyAvg/6.0

    #TODO Implement function for calculating clear path

if __name__ == "__main__":
    node = spacefinder()
    rospy.spin()
