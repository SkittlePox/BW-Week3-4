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
	self.right=-1

    def drive(self, angle, speed):

        ackmsg = AckermannDriveStamped()
       	ackmsg.drive.speed = speed
        ackmsg.drive.steering_angle = angle

        self.drivepub.publish(ackmsg)

    def coordconvert(self, radius, index):
	squrad=radius*radius
	angle=math.radians((index-180)/4)
	return (math.cos(angle)/squrad, math.sin(angle)/squrad)

    def parsescan(self, scan):
	
	vx=0.0;
	vy=0.0;

	valscale = .0006
	backcharge = .5
	steercoeff = 5
#	vmin = .1
#	stoprange=.4
#	backspeed=-.5

	for i in range(180,900):
	    vx-=self.coordconvert(scan.ranges[i], i)[0]
	    vy-=self.coordconvert(scan.ranges[i], i)[1]
	
	vx*=valscale
	vy*=valscale

	vyo = -(math.sqrt(vy*vy+vx*vx) + backcharge)*numpy.sign(vy)
	vxo = steercoeff*math.atan2(vx, vyo)*numpy.sign(vy)

#	if math.fabs(vyo)<vmin:
#	    vyo = -1
#	    vxo=self.right
#	    self.right*=-1
#	if min(scan.ranges[480:600])<stoprange:
#	    vxo=0
#	    vyo=backspeed

	self.drive(vxo, vyo)

	print str(vxo) + "," + str(vyo)
	   

if __name__ == '__main__':
	
    node = Fieldnav()
    rospy.spin()
