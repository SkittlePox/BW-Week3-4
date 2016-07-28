#!/usr/bin/env python

# general imports for all python nodes
import rospy
import math
import numpy
# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan # laser scanner msgs; Joy may be unnecessary

class safetystop:

    def __init__(self):

	rospy.init_node("safetystop")
	rospy.Subscriber('scan', LaserScan, self.parsescan)
	self.drivepub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/safety', AckermannDriveStamped, queue_size=1)	

    def parsescan(self, scan):

	if min(scan.ranges[0:1080])<.3:
	    self.drive(0,0)


if __name__ == '__main__':

    node = safetystop()
    rospy.spin()
