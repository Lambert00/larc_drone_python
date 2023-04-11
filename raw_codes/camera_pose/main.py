import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('obstacles/obstacle2.png')

hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
low = np.array([0, 96, 57], np.uint8)
high = np.array([21, 255, 255], np.uint8)
mask = cv2.inRange(hsv, low, high)

centroid = mask.shape[:2][::-1]
contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
# cv2.drawContours(img, contours, -1, (255, 0, 0), 2)
if len(contours)>0:
    contour = sorted(contours, key=cv2.contourArea)[0]
    epsilon = 0.1*cv2.arcLength(contour,True)
    approx = cv2.approxPolyDP(contour,epsilon,True)
    print(approx)
    if len(approx) == 4:
        for i in range(4): cv2.circle(img, approx[i][0], 10, (255,0,0), 2)
        mask_clean = np.zeros_like(mask, np.uint8)
        cv2.drawContours(mask_clean, [approx], 0, (255,255,255), -1)
        moment = cv2.moments(mask_clean)
        if moment['m00'] != 0: 
            centroid = moment['m10']/moment['m00'], moment['m01']/moment['m00']
            cv2.circle(img, tuple(map(round, centroid)), 2, (0,0,255), 2)

        points = np.zeros((4, 3))
        for i in range(4):
            th = np.arctan2(approx[i][0][1]-centroid[1], approx[i][0][0]-centroid[0])
            points[i] = np.array([approx[i][0][0], approx[i][0][1], th])
        
        points = np.array(sorted(points, key=lambda x: x[2], reverse=True))
        print(points)
        
        x,y,w,h = cv2.boundingRect(contour)
        cv2.rectangle(img, (x,y), (x+w, y+h), (0,255,255), 2)
        cv2.line(img, (x+w//2,y), (x+w//2, y+h), (255,0,0), 2)
        
        

cv2.imshow('mask', mask)
cv2.imshow('mask_clean', mask_clean)
cv2.imshow('img', img)
cv2.waitKey(0)
cv2.destroyAllWindows()