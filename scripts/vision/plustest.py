import cv2
import numpy as np

filters_green = [np.array([60, 100, 60]), np.array([88, 255, 255])]  # Green

plus = cv2.imread("plus.png", 0)
_,plus_binary = cv2.threshold(plus,127,255,cv2.THRESH_BINARY_INV)
contours_plus = cv2.findContours(plus_binary, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

test = cv2.imread("green.jpg")
print(test.shape)
test_hsv = cv2.cvtColor(test, cv2.COLOR_BGR2HSV)
mask_green = cv2.inRange(test_hsv, filters_green[0], filters_green[1])
contours_green = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]

if(len(contours_green) != 0):
    index2 = 0
    print(contours_green)
    for i in range(0, len(contours_green)):
        if(cv2.contourArea(contours_green[i]) > cv2.contourArea(contours_green[index2])):
            index2 = i
    the_one = contours_green[index2]

    if(len(contours_plus) != 0):
        index = 0
        for i in range(0, len(contours_plus)):
            if(cv2.contourArea(contours_plus[i]) > cv2.contourArea(contours_plus[index])):
                index = i
        the_plus = contours_green[index2]
        ret = cv2.matchShapes(the_one, the_plus, 1, 0.0)
        print(ret)

cv2.imshow("img",plus_binary)
cv2.waitKey(0)
cv2.destroyAllWindows()
