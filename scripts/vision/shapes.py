import cv2
import numpy as np

img2 = cv2.imread('plus.png')
img1 = cv2.imread('green.jpg')
image1_hsv = cv2.cvtColor(img1, cv2.COLOR_BGR2HSV)
image2_hsv = cv2.cvtColor(img2, cv2.COLOR_BGR2HSV)

filters_green = [np.array([60, 100, 60]), np.array([88, 255, 255])]  # Green
filters_blue = [np.array([106, 57, 41]), np.array([140, 255, 215])]
filters_black = [np.array([0, 0, 0]), np.array([0, 0, 1])]

mask_green = cv2.inRange(image1_hsv, filters_green[0], filters_green[1])
mask_black = cv2.inRange(image2_hsv, filters_black[0], filters_black[1])

contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
contours_black, _ = cv2.findContours(mask_black, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

def findLargest(contours):
    areas = []
    for i in range(0, len(contours)):
        area = cv2.contourArea(contours[i])
        areas.append(area)
    bestArea = max(areas)
    return contours[areas.index(bestArea)]

cnt1 = findLargest(contours_black)
cnt2 = findLargest(contours_green)

ret = cv2.matchShapes(cnt1, cnt2, 1, 0.0)

cv2.drawContours(img1, [cnt2], -1, (0, 255, 0), 3)
print(ret)
cv2.imshow("img", img1)
cv2.waitKey(0)
