#!/usr/bin/env python

# general imports for all python nodes
import rospy
import math
import numpy
# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped # steering messages
from sensor_msgs.msg import LaserScan # laser scanner msgs; Joy may be unnecessary

class Turnaround:

    def __init__(self):
        rospy.init_node("Turnaround")
      #  rospy.Subscriber('scan', LaserScan, self.parsescan)
        self.drivepub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)

    def turn(self):
        movesign = -1
        for i in range(0,100):
            self.drive(movesign, .5*movesign)
            rospy.sleep(1)
            movesign *=-1

    def drive(self, angle, speed): #publishes the appropriate drive method

        ackmsg = AckermannDriveStamped()
        ackmsg.drive.speed = speed
        ackmsg.drive.steering_angle = angle

        self.drivepub.publish(ackmsg)

  #  def parsescan(self, scan): #the main driving method



   #     self.drive(vxo, vyo) #drives

#	print str(vxo) + "," + str(vyo)


if __name__ == '__main__':

    node = Turnaround()
    rospy.spin()
    self.turn()
