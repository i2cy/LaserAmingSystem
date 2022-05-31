import cv2
import numpy as np
import time

# i2cy edit
from cam_config import CAM

cap = CAM
# end edit

# 帧640*480;亮度100/256
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_BRIGHTNESS, 100)

myColors = [[145, 75, 75, 185, 255, 255]]  # 颜色列表


def findColor(frame, myColors):
    imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    for color in myColors:
        lower = np.array(color[0:3])  # 下限
        upper = np.array(color[3:6])  # 上限
        mask = cv2.inRange(imgHSV, lower, upper)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)  # 查找轮廓

        rect_x, rect_y, rect_w, rect_h = 0, 0, 0, 0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 300:
                cv2.drawContours(imgResult, cnt, -1, (255, 0, 0), 3)
                peri = cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)
                rect_x, rect_y, rect_w, rect_h = cv2.boundingRect(approx)

        contours_x = rect_x + rect_w // 2
        contours_y = rect_y
        cv2.circle(imgResult, (contours_x, contours_y), 5, (0, 255, 0), cv2.FILLED)
        cv2.imshow("str(color[i])", mask)


T0 = 0
T1 = 0
while True:
    flag, frame = cap.read()
    if not flag:
        continue
    imgResult = frame.copy()
    findColor(frame, myColors)
    cv2.imshow("video", imgResult)
    T0 = T1
    T1 = time.time()
    fps = 1 // ((T1 - T0))
    print(fps)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
