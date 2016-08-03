#!/usr/bin/env python
import cv2
import rospy
import numpy as np
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import threading


class Recog:

    def __init__(self):
        self.node_name = "RecognizerORB"
        self.thread_lock = threading.Lock()

        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("/echo_image", Image, queue_size=1)
        self.pub_found = rospy.Publisher("/exploring_challenge", String, queue_size=1)
        self.pub_image_comp = rospy.Publisher("~echo_image/compressed", CompressedImage, queue_size=1)

        self.bridge = CvBridge()

        self.count = 0

        """
        plus = cv2.imread("plus.png", 0)
        _, plus_binary = cv2.threshold(plus, 127, 255, cv2.THRESH_BINARY_INV)
        _, contours_plus,_ = cv2.findContours(plus_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if(len(contours_plus) != 0):
            index = 0
            for i in range(0, len(contours_plus)):
                if(cv2.contourArea(contours_plus[i]) > cv2.contourArea(contours_plus[index])):
                    index = i
            self.plusCT = contours_plus[index]
        """
        self.the_time = rospy.Time.now()

    def cbImage(self, image_msg):
        thread = threading.Thread(target=self.processImage, args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, image_msg):
        try:
            self._processImage(image_msg)
        except Exception, e:
            print(str(e))

    def _processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
        display_text = " "
        # Image processing starts here

        the_one, color_scheme, display_text = self.color_search(image_cv)
        if not (the_one is None):  # If there is a blob
            #shape = self.plus_test(the_one)
            # Drawing contours
            cv2.drawContours(image_cv, [the_one], -1, (color_scheme.b, color_scheme.g, color_scheme.r))
            x, y, w, h = cv2.boundingRect(the_one)
            if(display_text == "pink"):
                display_text = self.image_classify(image_cv, the_one)
            #elif(shape is not None):
                #display_text += shape
            cv2.rectangle(image_cv, (x, y), (x + w, y + h), (color_scheme.b, color_scheme.g, color_scheme.r, 2))
            cv2.circle(image_cv, (x + w / 2, y + h / 2), 4, (255, 255, 255), -1)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image_cv, display_text, (x + w / 2, y + 3 * h / 4), font, 2, (255, 255, 255), 2)
        if(display_text is None):
            display_text = " "
        # Image processing stops here
        if (display_text is not " " and rospy.Time.now() - self.the_time >= rospy.Duration(2, 0)):
            self.pub_found.publish("Found {0}".format(display_text))
            cv2.imwrite("/home/racecar/challenge_photos/{0}{1}.png".format(display_text, self.count), image_cv)
            self.count += 1
            self.the_time = rospy.Time.now()

        # Compressing image to publish
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', image_cv)[1]).tostring()
        self.pub_image_comp.publish(msg)

        # Publishing uncompressed image
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))

        self.thread_lock.release()

    def plus_test(self, the_one):
        try:
            ret = cv2.matchShapes(the_one, self.plusCT, 1, 0.0)
            print(ret)
            if(ret < 0.1):
                return " plus"
        except:
            " noooo"
        return None

    def image_classify(self, image_cv, contour):
        train_ari = cv2.imread('/home/racecar/racecar-ws/src/come_on_and_SLAM/scripts/img/ari.jpg', 0)
        train_car = cv2.imread('/home/racecar/racecar-ws/src/come_on_and_SLAM/scripts/img/car.png', 0)
        train_cat = cv2.imread('/home/racecar/racecar-ws/src/come_on_and_SLAM/scripts/img/cat.png', 0)
        train_sertac = cv2.imread('/home/racecar/racecar-ws/src/come_on_and_SLAM/scripts/img/sertac.jpg', 0)
        orb = cv2.ORB()
        keypoints_ari, descriptors_ari = orb.detectAndCompute(train_ari, None)
        keypoints_car, descriptors_car = orb.detectAndCompute(train_car, None)
        keypoints_cat, descriptors_cat = orb.detectAndCompute(train_cat, None)
        keypoints_sertac, descriptors_sertac = orb.detectAndCompute(train_sertac, None)

        bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        image_gray = cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)
        x, y, w, h = cv2.boundingRect(contour)
        image_gray = image_gray[y:y + h, x:x + w]
        kps, dcs = orb.detectAndCompute(image_gray, None)
        try:
            matches_sertac = bf.match(descriptors_sertac, dcs)
            matches_ari = bf.match(descriptors_ari, dcs)
            matches_car = bf.match(descriptors_car, dcs)
            matches_cat = bf.match(descriptors_cat, dcs)
        except:
            return " "
        # print(matches_sertac)

        matches_ari = sorted(matches_ari, key=lambda x: x.distance)
        matches_sertac = sorted(matches_sertac, key=lambda x: x.distance)
        matches_car = sorted(matches_car, key=lambda x: x.distance)
        matches_cat = sorted(matches_cat, key=lambda x: x.distance)

        matches = [matches_ari, matches_sertac, matches_cat, matches_car]
        avgs = [0, 0, 0, 0]   # ari, sertac, cat, car
        counters = [0, 0, 0, 0]
        objs = ["ari", "sertac", "cat", "racecar"]

        for i in range(0, len(matches)):
            for j in range(0, len(matches[i])):
                avgs[i] += matches[i][j].distance
                counters[i] += 1
                if(counters[i] == 9):
                    avgs[i] /= counters[i]
                    avgs[i] /= (10 - counters[i])
                    break
            if(len(matches[i]) == 0):
                avgs[i] = 1000000000
        minval = min(avgs)
        for i in range(0, len(matches)):
            if(avgs[i] == minval):
                return objs[i]

        # Comment out when passed
        calc_ari = len(matches_ari) != 0
        calc_sertac = len(matches_sertac) != 0
        calc_car = len(matches_car) != 0
        calc_cat = len(matches_cat) != 0

        if(len(matches_ari) == 0):
            if(len(matches_sertac) is not 0):
                return "sertac"
            else:
                return "idk1"
        if(len(matches_sertac) == 0):
            if(len(matches_ari) is not 0):
                return "ari"
            else:
                return "idk2"

        avg_ari = 0
        avg_sertac = 0
        counter_a = 0
        counter_s = 0
        for i in range(0, len(matches_ari)):
            avg_ari += matches_ari[i].distance
            counter_a += 1
            if (counter_a == 9):
                break

        for i in range(0, len(matches_sertac)):
            avg_sertac += matches_sertac[i].distance
            counter_s += 1
            if (counter_s == 9):
                break

        avg_ari /= counter_a
        avg_sertac /= counter_s

        if(avg_ari < avg_sertac):
            return "ari"
        else:
            return "sertac"

    def color_search(self, image_cv):
        image_hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

        filters_red = [np.array([0, 165, 70]), np.array([6, 255, 255])]  # Red
        filters_red2 = [np.array([170, 165, 140]), np.array([180, 255, 255])]  # Red
        filters_green = [np.array([50, 100, 30]), np.array([88, 255, 255])]  # Green
        filters_yellow = [np.array([22, 150, 84]), np.array([40, 255, 190])]  # Yellow
        filters_blue = [np.array([106, 127, 40]), np.array([127, 235, 215])]  # Blue
        filters_pink = [np.array([144, 91, 116]), np.array([165, 145, 255])]  # Pink

        mask_red = cv2.inRange(image_hsv, filters_red[0], filters_red[1])
        mask_red2 = cv2.inRange(image_hsv, filters_red2[0], filters_red2[1])
        mask_green = cv2.inRange(image_hsv, filters_green[0], filters_green[1])
        mask_yellow = cv2.inRange(image_hsv, filters_yellow[0], filters_yellow[1])
        mask_blue = cv2.inRange(image_hsv, filters_blue[0], filters_blue[1])
        mask_pink = cv2.inRange(image_hsv, filters_pink[0], filters_pink[1])

        # Bridging two red filters
        mask_red = cv2.bitwise_or(mask_red, mask_red2)

        mask_red = cv2.GaussianBlur(mask_red, (3, 3), 0)
        mask_green = cv2.GaussianBlur(mask_green, (3, 3), 0)
        mask_yellow = cv2.GaussianBlur(mask_yellow, (3, 3), 0)
        mask_blue = cv2.GaussianBlur(mask_blue, (3, 3), 0)
        mask_pink = cv2.GaussianBlur(mask_pink, (3, 3), 0)

        contours_red = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        contours_green = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        contours_yellow = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        contours_blue = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        contours_pink = cv2.findContours(mask_pink, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

        test_red = len(contours_red) > 0
        test_green = len(contours_green) > 0
        test_yellow = len(contours_yellow) > 0
        test_blue = len(contours_blue) > 0
        test_pink = len(contours_pink) > 0

        if not (test_red or test_green or test_yellow or test_blue or test_pink):  # TODO implement clock check with time.clock() - 5 > the_time, omitted for debugging purposes
            return

        contours = [contours_red, contours_green, contours_yellow, contours_blue, contours_pink]
        tests = [test_red, test_green, test_yellow, test_blue, test_pink]
        colors = ["red", "green", "yellow", "blue", "pink"]
        color_objects = [std_msgs.msg.ColorRGBA(255.0, 0.0, 0.0, 0.0), std_msgs.msg.ColorRGBA(0.0, 255.0, 0.0, 0.0), std_msgs.msg.ColorRGBA(255.0, 255.0, 0.0, 0.0), std_msgs.msg.ColorRGBA(0.0, 0.0, 255.0, 0.0), std_msgs.msg.ColorRGBA(255.0, 0.0, 255.0, 0.0)]

        # Populated Later
        largest_areas = []
        largest_contours = []

        # Winning contour specific variables
        the_one = None  # The winning contour
        display_text = ""
        color_scheme = None  # What color box to draw around it

        for i in range(0, len(tests)):
            if(tests[i]):   # If any contours were found of the 'i'th color
                max_contour = contours[i][0]
                for j in range(0, len(contours[i])):
                    if(cv2.contourArea(contours[i][j]) > cv2.contourArea(max_contour)):
                        max_contour = contours[i][j]
                if(i == 4):  # Pink
                    largest_areas.append(cv2.contourArea(max_contour) * 1.5)
                else:
                    largest_areas.append(cv2.contourArea(max_contour))  # Place the largest found contour's area in the largest_areas array
                largest_contours.append(max_contour)                # Place the contour itself in the largest_contours array
            else:   # If no contours exist for a given mask populated it with empty values
                largest_areas.append(0)
                largest_contours.append(np.array([[0, 0]]))

        largest = max(largest_areas)
        for i in range(0, len(largest_areas)):  # Finds the winning contour
            if(largest_areas[i] == largest):    # This is the winning contour TODO implement threshold for size!
                the_one = largest_contours[i]
                display_text = colors[i]
                color_scheme = color_objects[i]
        if (largest < 7000):
            return None, None, None

        return the_one, color_scheme, display_text

if __name__ == "__main__":
    rospy.init_node('RecognizerORB')
    main = Recog()
    rospy.spin()
