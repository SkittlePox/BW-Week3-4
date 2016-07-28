#!/usr/bin/env python
import cv2
import rospy
import numpy as np
import time
from sensor_msgs.msg import Image
from std_msgs.msg import *
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Point
import threading


class Recog:

    def __init__(self):
        self.node_name = "Recog"
        self.thread_lock = threading.Lock()
        self.sub_image = rospy.Subscriber("/camera/rgb/image_rect_color", Image, self.cbImage, queue_size=1)
        self.pub_image = rospy.Publisher("/echo_image", Image, queue_size=1)
        self.pub_found = rospy.Publisher("/exploring_challenge", String, queue_size=1)
        self.bridge = CvBridge()
	self.count = 0
        self.the_time = 0

    def cbImage(self, image_msg):
        thread = threading.Thread(target=self.processImage, args=(image_msg,))
        thread.setDaemon(True)
        thread.start()

    def processImage(self, image_msg):
        if not self.thread_lock.acquire(False):
            return
        image_cv = self.bridge.imgmsg_to_cv2(image_msg)
	display_text = " "
        # Image processing starts here
        #face = self.face_search(image_cv)

        if (False):#(face is not None):
            # Drawing rectangle for face
            display_text = self.faceClasify(face, image_cv)
            cv2.rectangle(image_cv, (face[0], face[1]), (face[0] + face[2], face[1] + face[3]), (0, 255, 0), 2)
            font = cv2.FONT_HERSHEY_SIMPLEX
            cv2.putText(image_cv, display_text, (face[0] + face[0] / 2, face[1] + 3 * face[3] / 4), font, 4, (255, 255, 255), 2)
        else:
            the_one, color_scheme, display_text = self.color_search(image_cv)
	    if not (the_one is None):
            	# Drawing contours
            	cv2.drawContours(image_cv, [the_one], -1, (color_scheme.b, color_scheme.g, color_scheme.r))
            	x, y, w, h = cv2.boundingRect(the_one)
            	cv2.rectangle(image_cv, (x, y), (x + w, y + h), (color_scheme.b, color_scheme.g, color_scheme.r, 2))
            	cv2.circle(image_cv, (x + w / 2, y + h / 2), 4, (255, 255, 255), -1)
            	font = cv2.FONT_HERSHEY_SIMPLEX
            	cv2.putText(image_cv, display_text, (x + w / 2, y + 3 * h / 4), font, 2, (255, 255, 255), 2)
	if(display_text is "None"):
	    display_text = " "
        # Image processing stops here
	if (display_text is not " " and time.time() - 2 >= self.the_time):
            self.pub_found.publish("Found {0}".format(display_text))
            cv2.imwrite("{0}{1}.png".format(display_text, self.count), image_cv)
	    self.count += 1
	    self.the_time = time.time()
	    print(time.time()-1>self.the_time)
        self.pub_image.publish(self.bridge.cv2_to_imgmsg(image_cv, "bgr8"))

        self.thread_lock.release()

    def face_search(self, image_cv):
        faceCascade = cv2.CascadeClassifier("haarcascade_frontalface_default.xml")
        gray = cv2.cvtColor(image_cv, cv2.COLOR_BGR2GRAY)
        faces = faceCascade.detectMultiScale(   # Finds faces
            gray,
            scaleFactor=1.2,
            minNeighbors=5,
            minSize=(30, 30),
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        if(len(faces) == 0):
            return None

        maxIndex = 0
        index = 0
        for (x, y, w, h) in faces:  # Finds largest face
            if(h * w > faces[maxIndex][3] * faces[maxIndex][2]):
                maxIndex = index
            index += 1

        the_face = faces[maxIndex]
        return the_face

    def faceClasify(self, the_face, image_cv):
        x = the_face[0]
        y = the_face[1]
        w = the_face[2]
        h = the_face[3]

        cv2.rectangle(image_cv, (x, y), (x + w, y + h), (0, 255, 0), 2)

        image_crop = image_cv[y:y+h, x:x+w]    # Just the face
        image_hsv = cv2.cvtColor(image_crop, cv2.COLOR_BGR2HSV)

        filters_ari = [np.array([8, 110, 100]), np.array([12, 255, 225])]  # Ari
        filters_sertac = [np.array([2, 0, 0]), np.array([9, 115, 255])]  # Sertac

        mask_ari = cv2.inRange(image_hsv, filters_ari[0], filters_ari[1])
        mask_sertac = cv2.inRange(image_hsv, filters_sertac[0], filters_sertac[1])

        sertac_count = 0
        ari_count = 0

        for i in range(0, len(mask_sertac)):
            for j in range(0, len(mask_sertac[i])):
                if(mask_sertac[i][j] > 127):    # White pixel
                    sertac_count += 1
                else:
                    sertac_count -= 1
        #print(sertac_count)

        for i in range(0, len(mask_ari)):
            for j in range(0, len(mask_ari[i])):
                if(mask_ari[i][j] > 127):    # White pixel
                    ari_count += 1
                else:
                    ari_count -= 1
        #print(ari_count)
        who = ""
        if(sertac_count > ari_count):
            who = "sertac"
        else:
            who = "ari"

        return who

    def color_search(self, image_cv):
        image_hsv = cv2.cvtColor(image_cv, cv2.COLOR_BGR2HSV)

        filters_red = [np.array([0, 165, 100]), np.array([6, 255, 255])]  # Red
        filters_red2 = [np.array([170, 165, 170]), np.array([180, 255, 255])]  # Red
        filters_green = [np.array([60, 100, 60]), np.array([88, 255, 255])]  # Green
        filters_yellow = [np.array([22, 150, 114]), np.array([33, 255, 190])]  # Yellow
        filters_blue = [np.array([106, 127, 51]), np.array([127, 255, 215])]  # Blue

        mask_red = cv2.inRange(image_hsv, filters_red[0], filters_red[1])
        mask_red2 = cv2.inRange(image_hsv, filters_red2[0], filters_red2[1])
        mask_green = cv2.inRange(image_hsv, filters_green[0], filters_green[1])
        mask_yellow = cv2.inRange(image_hsv, filters_yellow[0], filters_yellow[1])
        mask_blue = cv2.inRange(image_hsv, filters_blue[0], filters_blue[1])

        # Bridging two red filters
        mask_red = cv2.bitwise_or(mask_red, mask_red2)

        mask_red = cv2.GaussianBlur(mask_red, (3, 3), 0)
        mask_green = cv2.GaussianBlur(mask_green, (3, 3), 0)
        mask_yellow = cv2.GaussianBlur(mask_yellow, (3, 3), 0)
        mask_blue = cv2.GaussianBlur(mask_blue, (3, 3), 0)

        contours_red = cv2.findContours(mask_red, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        contours_green = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        contours_yellow = cv2.findContours(mask_yellow, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        contours_blue = cv2.findContours(mask_blue, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

        test_red = len(contours_red) > 0
        test_green = len(contours_green) > 0
        test_yellow = len(contours_yellow) > 0
        test_blue = len(contours_blue) > 0

        if not (test_red or test_green or test_yellow or test_blue):  # TODO implement clock check with time.clock() - 5 > the_time, omitted for debugging purposes
            return

        contours = [contours_red, contours_green, contours_yellow, contours_blue]
        tests = [test_red, test_green, test_yellow, test_blue]
        colors = ["red", "green", "yellow", "blue"]
        color_objects = [std_msgs.msg.ColorRGBA(0.0, 0.0, 255.0, 0.0), std_msgs.msg.ColorRGBA(0.0, 255.0, 0.0, 0.0), std_msgs.msg.ColorRGBA(255.0, 255.0, 0.0, 0.0), std_msgs.msg.ColorRGBA(255.0, 0.0, 0.0, 0.0)]

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
                self.the_time = time.clock()
	if (cv2.contourArea(the_one) < 10000):
	    return None, None, None

        return the_one, color_scheme, display_text

if __name__ == "__main__":
    rospy.init_node('Recog')
    main = Recog()
    rospy.spin()
