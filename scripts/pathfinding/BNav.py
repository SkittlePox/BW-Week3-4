import rospy
import math
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from sensor_msgs.msg import LaserScan


class BNav:

    def __init__(self):
        rospy.init_node("BNav")
        rospy.Subscriber('scan', LaserScan, self.nav)
        self.drivepub = rospy.Publisher(
            '/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped, queue_size=1)

    def drive(self, angle, speed):
        dmsg = AckermannDriveStamped()
        dmsg.drive.speed = speed
        dmsg.drive.steering_angle = angle

        self.drivepub.publish(dmsg)

    def nav(self, msg):     # Main navigation function
        print(calcResultantVector(msg.ranges))

    def calcResultantVector(self, ranges):  # Translates each LIDAR value into a vector and then adds them
        cX = 0  # Cartesian X
        cY = 0  # Cartesian Y

        for i in range(0, len(ranges)):
            cX += (0.1 / ranges[i]**2) * np.sin(np.deg2rad((i - 540) / 4))
            cY += -1 * (0.1 / ranges[i]**2) * np.cos(np.deg2rad(180 - (i - 540) / 4))
        cY += 5

        return np.rad2deg(np.atan2(cY, cX))


if __name__ == '__main__':
    node = BNav()
    rospy.spin()
