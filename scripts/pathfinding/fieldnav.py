#!/usr/bin/env python

# general imports for all python nodes
import rospy
import math
import numpy
# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan # laser scanner msgs; Joy may be unnecessary

class Fieldnav:

    def __init__(self):
	rospy.init_node("Fieldnav")
	rospy.Subscriber('scan', LaserScan, self.parsescan)
	self.drivepub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)

    def drive(self, angle, speed):

        ackmsg = AckermannDriveStamped()
       	ackmsg.drive.speed = speed
        ackmsg.drive.steering_angle = angle

        self.drivepub.publish(ackmsg)

    def coordconvert(self, radius, index):
	squrad=radius*radius
	angle=math.radians((index-180)/6)
	return (math.cos(angle)/squrad, math.sin(angle)*math.sin(angle)/squrad)

    def parsescan(self, scan):
	
	vx=0.0;
	vy=0.0;

	for i in range(180,900):
	    vx+=self.coordconvert(scan.ranges[i], i)[0]
	    vy+=self.coordconvert(scan.ranges[i], i)[1]

	self.drive(vx*.001, (vy*-.0006)+1.2)
	print str(vx*.001) + "," + str(vy*.0006)
	   

if __name__ == '__main__':
	
    node = Fieldnav()
    rospy.spin()
