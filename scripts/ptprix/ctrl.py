#!/usr/bin/env python

import rospy
import math
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import *
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32MultiArray


class Controller:
    def __init__(self):
        rospy.Subscriber('scan', LaserScan, self.scanReceived)
        rospy.Subscriber('detect', Int32MultiArray, self.detectReceived)
        self.drivepub = rospy.Publisher(
            '/vesc/ackermann_cmd_mux/input/navigation', AckermannDriveStamped,
            queue_size=1)

        self.charge_laser_particle = 0.07

        self.boost_distance = 0.5
        self.speed_const = 3.0
        self.p_speed = 0.02
        self.p_steering = 1.5
        self.run = True

        self.last_y = 0
        self.Kd = 0.2

        self.x_components = {"backCharge": 200.0}
        self.y_components = {"leftCharge": 0.0}

    def detectReceived(self, msg):
        x, area, color = msg.data
        coeff = 300
        print("ye")
        print(x, area, color)
        if area > 700 and color != -1:
            if color == 0:
                self.y_components['leftCharge'] = -coeff
                print("right")
            elif color == 1:
                self.y_components['leftCharge'] = coeff-100
                print("strsaight")
        else:
            print("non")
            self.y_components['leftCharge'] = 0.0

    def scanReceived(self, msg):
        scan_rad_angles = ((msg.angle_increment *
                            np.arange(1081, dtype=float)) + msg.angle_min)

        scan_x_unit_vectors = -np.cos(scan_rad_angles)
        scan_y_unit_vectors = -np.sin(scan_rad_angles)

        scan_x_components = (self.charge_laser_particle *
                             scan_x_unit_vectors) / np.sqrt(msg.ranges)
        scan_y_components = (self.charge_laser_particle *
                             scan_y_unit_vectors) / np.sqrt(msg.ranges)

        total_x_component = np.sum(scan_x_components) + sum(
            self.x_components.values())
        # print("x/sum", total_x_component, sum(self.x_components.values()))
        total_y_component = np.sum(scan_y_components) + sum(
            self.y_components.values())

        angle = ((self.p_steering * np.sign(total_x_component) * math.atan2(
            total_y_component, total_x_component)) + (self.Kd * (
                total_y_component - self.last_y)))

        # speed = (self.p_speed * np.sign(total_x_component) * math.sqrt(
        #    total_x_component**2 + total_y_component**2))
        speed = self.speed_const
        # speed = self.p_speed*total_x_component

        self.last_y = total_y_component
        print(total_x_component)
        self.drive(angle, speed)

    def drive(self, angle, speed):
        if(self.run):
            ackmsg = AckermannDriveStamped()
            ackmsg.drive.speed = speed
            ackmsg.drive.steering_angle = angle
            self.drivepub.publish(ackmsg)

    def handle_joy(self, msg):
        if(msg.buttons[3] == 1):    # A button
            self.run = True

if __name__ == '__main__':
    rospy.init_node('Controller')
    node = Controller()
    rospy.spin()
