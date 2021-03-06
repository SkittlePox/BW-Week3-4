#!/usr/bin/env python

# general imports for all python nodes
import rospy
import math
import numpy
# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan, Joy # joystick and laser scanner msgs; Joy may be unnecessary

class Bugnav:

    def __init__(self):
	
	rospy.Subscriber('scan', LaserScan, self.parsescan)
	self.drivepub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)

    def drive(self, angle, speed):

        ackmsg = AckermannDriveStamped()
       	ackmsg.drive.speed = speed
        ackmsg.drive.steering_angle = angle

        self.drivepub.publish(ackmsg)

    def parsescan(self, scan):

	if min(scan.ranges[400:680])>.5:
	    self.drive(0,.5)
	else:
	    if min(scan.ranges[0:400])>min(scan.ranges[680:1080]):
		self.drive(.5,.5)
	    else:
		self.drive(-.5,.5)

if __name__ == '__main__':
	
    node = Bugnav()
    rospy.init_node("Bugnav")
    rospy.spin()

