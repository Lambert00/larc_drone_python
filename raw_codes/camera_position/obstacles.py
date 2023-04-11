import cv2
import numpy as np

class Obstacle():
    def __init__(self):
        # self.low = np.array([0, 121, 54], np.uint8)
        # self.high = np.array([24, 255, 255], np.uint8)
        self.low = np.array([0, 138, 54], np.uint8)
        self.high = np.array([17, 255, 255], np.uint8)
    
    def get_vertices(self, hsv):
        points = np.zeros((4, 3), np.float32)
        mask = cv2.inRange(hsv, self.low, self.high)
        mask_clean = np.zeros_like(mask, np.uint8)
        contours = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)[0]
        if len(contours)>0:
            contour = sorted(contours, key=cv2.contourArea, reverse=True)[0]
            epsilon = 0.1*cv2.arcLength(contour,True)
            approx = cv2.approxPolyDP(contour,epsilon,True)
            if len(approx) != 4: return False, points 
            cv2.drawContours(mask_clean, [approx], 0, (255,255,255), -1)
            moment = cv2.moments(mask_clean)
            if moment['m00'] == 0: return False, points
            centroid = moment['m10']/moment['m00'], moment['m01']/moment['m00']
            for i in range(4):
                th = np.arctan2(approx[i][0][1]-centroid[1], approx[i][0][0]-centroid[0])
                points[i] = np.array([approx[i][0][0], approx[i][0][1], th])
            points = np.array(sorted(points, key=lambda x: x[2]))
            cv2.imshow('mask', mask)
            cv2.imshow('mask_clean', mask_clean)
        return True, np.array(points[:,:2], np.float32)