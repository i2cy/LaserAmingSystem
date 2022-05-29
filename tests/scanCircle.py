import numpy as np
import cv2
import time

cap = cv2.VideoCapture(
    'rkisp device=/dev/video1 io-mode=4 ! video/x-raw,format=NV12,width=640,height=480,framerate=120/1 ! videoconvert '
    '! appsink',
    cv2.CAP_GSTREAMER)
count = 0
T0 = 0
T1 = 0
while 1:
    flag, frame = cap.read()
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # gray_frame = cv2.medianBlur(gray_frame,5)
    circles = cv2.HoughCircles(gray_frame, cv2.HOUGH_GRADIENT, 1, 20, param1=50, param2=30, minRadius=5, maxRadius=60)
    if circles is None:
        continue
    if circles.size != 0:
        circles = np.uint16(np.around(circles))
        for i in circles[0, :]:
            cv2.circle(frame, (i[0], i[1]), i[2], (0, 255, 0), 2)
            # 作圆心
            cv2.circle(frame, (i[0], i[1]), 2, (0, 0, 255), 3)
        cv2.imshow('detected circles', frame)
        T0 = T1
        T1 = time.time()
        fps = 1 // (T1 - T0)
        print(fps)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cv2.destroyAllWindows()
print(circles)
