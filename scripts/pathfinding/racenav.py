#!/usr/bin/env python

# general imports for all python nodes
import rospy
import math
import numpy
# node specific imports
from ackermann_msgs.msg import AckermannDriveStamped  # steering messages
from sensor_msgs.msg import LaserScan  # laser scanner msgs; Joy may be unnecessary


class Fieldexplore:

    def __init__(self):
        rospy.init_node("Fieldexplore")
        rospy.Subscriber('scan', LaserScan, self.parsescan)
        self.drivepub = rospy.Publisher('/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)

        self.maxspeed = .8    # maximum speed
        self.basebackcharge = 1.1  # base value of backcharge
        self.backcharge = self.basebackcharge  # fixed charge pushing car
        self.mincharge = -.1    # minimum

        self.maxderiv = .0001  # maximum acceleration
        self.lastspeed = 0    # previous speed, for calculating acceleration

    def drive(self, angle, speed):  # publishes the appropriate drive method

        ackmsg = AckermannDriveStamped()
        ackmsg.drive.speed = speed
        ackmsg.drive.steering_angle = angle

        self.drivepub.publish(ackmsg)

    def capcharge(self, charge):  # causes charge pulling car back to plateau and act as a timer, keeps car from backing up too fast
        if charge < self.mincharge:
            return self.mincharge  # caps charge
        else:
            return charge

    def capspeed(self, speed):

        if speed > self.maxspeed:
            if self.maxspeed-self.lastspeed > self.maxderiv and self.lastspeed > .4:  # checks if capping to maxspeed would still exceed derivative limit
                return self.lastspeed + self.maxderiv  # caps by derivative when speed exceeds derivative and speed limits and exceeds derivative limit by more
            return self.maxspeed  # caps speed when it exceeds maximum by more than derivative
        elif -speed > self.maxspeed:  # caps backing up speed
            return -self.maxspeed
        elif speed - self.lastspeed > self.maxderiv and self.lastspeed > .4:  # caps by derivative
            return self.lastspeed + self.maxderiv
        else:
            return speed  # doesn't change speed

    def coordconvert(self, radius, index):  # converts lidar points to cartesian force vectors via inverse square charges
        squrad = radius*radius
        angle = math.radians((index-180)/4)
        return (math.cos(angle)/squrad, math.sin(angle)/squrad)

    def parsescan(self, scan):  # the main driving method

        vx = 0.0
        vy = 0.0  # net force vector without artificial charge

        valscale = .0006  # charge strength
        steercoeff = -10  # steering coefficient
        vmin = .25  # speed at which the car considers itself stopped

        for i in range(180, 900):  # adds up force vectors
            vx -= self.coordconvert(scan.ranges[i], i)[0]
            vy -= self.coordconvert(scan.ranges[i], i)[1]

        vx *= valscale  # scales net force vector by charge strength
        vy *= valscale

        #	vyo = -(math.sqrt(vy*vy+vx*vx) + backcharge)*numpy.sign(vy)
        #	vxo = steercoeff*math.atan2(vx, vyo)*numpy.sign(vy)

        vyo = vy+self.backcharge  # adds artificial charge
        vxo = steercoeff*math.atan2(vx, vyo)  # computes steering angle

        if vyo < vmin and -3 < vyo:  # backs up when stopped for a fixed time
            self.backcharge -= .1
            vyo = vy+self.capcharge(self.backcharge)
            vxo *= -1

        else:
            self.backcharge = self.basebackcharge  # stops backing up

        vyo = self.capspeed(vyo)  # caps speed

        self.lastspeed = vyo  # sets lastspeed to calculate derivative

        self.drive(vxo, vyo)  # drives


if __name__ == '__main__':

    node = Fieldexplore()
    rospy.spin()
