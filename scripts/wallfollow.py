#!/usr/bin/python

import rospy
from ackermann_msgs.msg import *
from sensor_msgs.msg import *
from nav_msgs.msg import *
import sys, math
from std_msgs.msg import Float32MultiArray

class ObjectDetectorNode:

    """
    When launching this rospy node, be sure to enter the following arguments in the order listed:\n
    **P-value:** for using proportion to tune the PID control loop\n
    **I-value:** for using integration to tune the PID control loop\n
    **D-value:** for using differential to tune the PID control loop\n
    **Joystick debounce time:** how long ROS should wait before it resumes listening to the wall-switching button (in secs)\n
    NOTE: A-button switches walls and B-button stops the car permanently

    """

    def __init__(self):

        self.refresh_count = 0
        self.refresh_count = rospy.get_param('refresh_count', default=self.refresh_count)
        self.desired_distance = 0.6
        self.desired_distance = rospy.get_param('desired_distance', default=self.desired_distance)
        self.threshold = 0.07
        self.desired_distance = rospy.get_param('desired_distance', default=self.desired_distance)
        self.rolling_sum = 0

        self.before_e = 0
        self.is_running = False
        self.is_right = True
        self.stopped = False

        self.drive_pub = rospy.Publisher("/vesc/ackermann_cmd_mux/input/navigation", AckermannDriveStamped)

        self.vision = rospy.Subscriber("/scan", LaserScan, self.drive_control)
        self.joystick = rospy.Subscriber("/vesc/joy", Joy, self.handle_buttons)

        rospy.init_node("object_detector_node")
        self.header = std_msgs.msg.Header()
        self.header.stamp = rospy.Time.now()
        self.STOP = AckermannDriveStamped(self.header, AckermannDrive(steering_angle=0.0, speed=0.0))
        rospy.loginfo("Moving forward...")

    def handle_buttons(self, msg):
        a_pressed = msg.buttons[0]
        b_pressed = msg.buttons[1]
	third_but_pressed = msg.buttons[3]
        if a_pressed == 1:
            self.is_right = not self.is_right
            rospy.sleep(float(sys.argv[4]))
        if b_pressed == 1:
            self.stopped = True
        if third_but_pressed == 1:
	    self.is_running = True	

    def drive_control(self, msg):
        if self.is_running:
            if self.stopped:
                self.drive_pub.publish(self.STOP)
	        rospy.loginfo("Car Stopped")
            else:
	        if self.is_right:
	            rospy.loginfo("Tracking right wall")
	        else:
	    	    rospy.loginfo("Tracking left wall")
                self.refresh_count += 1
                if self.is_right:
                    pts = msg.ranges[170:200]
                    pt_one = msg.ranges[180]
                    pt_two = msg.ranges[300]
                else:
                    pts = msg.ranges[880:910]
                    pt_one = msg.ranges[900]
                pt_two = msg.ranges[780]
                error = self.desired_distance - min(pts)
                d_hat = ((pt_one*pt_two)/(2*math.sqrt((math.pow(pt_one, 2) + math.pow(pt_two, 2) - math.sqrt(3)*pt_one*pt_two))))
                error_two = self.desired_distance - d_hat
	        error_new = (error + error_two)/2
                if self.refresh_count == 1:
                    self.before_e = error
            # new_steering_angle = self.bang_bang_controller(error)
            # new_steering_angle = self.bang_bang_with_threshold(error)
                new_steering_angle = self.pid_controller(error_new)
                drive_command = AckermannDriveStamped(self.header, AckermannDrive(steering_angle=new_steering_angle, speed=2.0))
                self.drive_pub.publish(drive_command)

    def bang_bang_controller(self, error):
        rospy.loginfo(error)
        if error < 0:
            if self.is_right:
                return -1
            else:
                return 1
        elif error > 0:
            if self.is_right:
                return 1
            else:
                return 0
        else:
            return 0

    def bang_bang_with_threshold(self, error):
        rospy.loginfo(error)
        if abs(error) > self.threshold:
            if error < -self.threshold:
                if self.is_right:
                    return -1
                else:
                    return 1
            else:
                if self.is_right:
                    return 1
                else:
                    return -1
        else:
            return 0

    def pid_controller(self, error):
        rospy.loginfo(error)
        return self.proportion(error, self.is_right) + self.differential(error, 40.0, self.is_right) + self.integral(error, self.is_right)

    def proportion(self, error, is_right):
        if is_right:
            return float(sys.argv[1]) * float(error)
        else:
            return -1 * float(sys.argv[1]) * float(error)

    def differential(self, error, hz, is_right):
        delta_time = 1 / hz
        edot = float(error - self.before_e) / float(delta_time)
        kd = float(sys.argv[3])
        self.before_e = error
        if math.fabs(kd) == 0:
	    return 0.0
	if is_right:
            return kd * edot
        else:
            return -kd * edot

    def integral(self, error, is_right):
        self.rolling_sum += error
        ki = float(sys.argv[2])
        if is_right:
            return ki * self.rolling_sum
        else:
            return -1 * ki * self.rolling_sum

if __name__ == "__main__":

    node = ObjectDetectorNode()

    rospy.spin()
