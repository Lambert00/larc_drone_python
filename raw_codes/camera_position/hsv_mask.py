import cv2
import numpy as np

dst = cv2.imread('outlet/img2.jpg')

hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
cv2.imshow('dst', dst)

def nothing(value): pass
cv2.namedWindow('settings_hsv')

cv2.createTrackbar('HL', 'settings_hsv', 0, 180, nothing)
cv2.createTrackbar('SL', 'settings_hsv', 0, 255, nothing)
cv2.createTrackbar('VL', 'settings_hsv', 0, 255, nothing)

cv2.createTrackbar('HH', 'settings_hsv', 0, 180, nothing)
cv2.createTrackbar('SH', 'settings_hsv', 0, 255, nothing)
cv2.createTrackbar('VH', 'settings_hsv', 0, 255, nothing)

while True:
    HL = cv2.getTrackbarPos('HL', 'settings_hsv')
    SL = cv2.getTrackbarPos('SL', 'settings_hsv')
    VL = cv2.getTrackbarPos('VL', 'settings_hsv')

    HH = cv2.getTrackbarPos('HH', 'settings_hsv')
    SH = cv2.getTrackbarPos('SH', 'settings_hsv')
    VH = cv2.getTrackbarPos('VH', 'settings_hsv')

    low = np.array([HL, SL, VL], np.uint8)
    high = np.array([HH, SH, VH], np.uint8)

    color_low = np.empty((100,100,3), np.uint8)
    color_low[:,:,0]=low[0]
    color_low[:,:,1]=low[1]
    color_low[:,:,2]=low[2]
    color_low = cv2.cvtColor(color_low, cv2.COLOR_HSV2BGR)

    color_high = np.empty((100,100,3), np.uint8)
    color_high[:,:,0]=high[0]
    color_high[:,:,1]=high[1]
    color_high[:,:,2]=high[2]
    color_high = cv2.cvtColor(color_high, cv2.COLOR_HSV2BGR)

    combinate = np.concatenate((color_low, color_high), axis=1)
    cv2.imshow('settings_hsv', combinate)
    hsv = cv2.cvtColor(dst, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    cv2.imshow('mask', mask)
    if cv2.waitKey(1) == 27: break

print(f"self.low = np.array({[HL, SL, VL]}, np.uint8)")
print(f"self.high = np.array({[HH, SH, VH]}, np.uint8)")
# print([HL, SL, VL],[HH, SH, VH])
cv2.destroyAllWindows()