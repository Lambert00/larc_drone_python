import cv2
import numpy as np
import time

from parrot_camera import *
from obstacles import *

camera = cv2.VideoCapture('video.mp4')
pcam = ParrotCamera(0.118, 0.075)
obs = Obstacle()

first = time.time()

while camera.isOpened():
    last = time.time()
    print(1/(last-first))
    first = last
    ret, frame = camera.read()
    if ret == False: break
    
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV) 
    retobs, points_2D = obs.get_vertices(hsv)
    
    if retobs: 
        rvec, tvec = pcam.get_pose(points_2D)
        pcam.project_3D(-rvec, tvec, points_2D[0], frame)

        print(f"x: {tvec[0]},\t y: {tvec[1]},\t z: {tvec[2]}")
        print(f"phi: {rvec[0]},\t theta: {rvec[1]},\t yaw: {rvec[2]}\n")
    
    cv2.imshow('frame', frame)
    if cv2.waitKey(33) == 27: break
    
camera.release()
cv2.destroyAllWindows()