import numpy as np
import cv2

cap = cv2.VideoCapture(
    'rkisp device=/dev/video1 io-mode=4 ! video/x-raw,format=NV12,width=640,height=480,framerate=120/1 ! videoconvert '
    '! appsink',
    cv2.CAP_GSTREAMER)
while 1:
    flag, frame = cap.read()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = np.float32(gray)
    dst = cv2.cornerHarris(gray, 2, 25, 0.04)
    # result is dilated for marking the corners, not important
    dst = cv2.dilate(dst, None)
    # Threshold for an optimal value, it may vary depending on the image.
    frame[dst > 0.01 * dst.max()] = [0, 0, 255]

    frame0 = frame.copy()
    corners = cv2.goodFeaturesToTrack(gray, 4, 0.01, 10)
    corners = np.int0(corners)
    for i in corners:
        x, y = i.ravel()
        cv2.circle(frame0, (x, y), 3, 255, -1)

    cv2.imshow('shi', frame0)
    #    cv2.imshow('dst',frame)
    if cv2.waitKey(1) & 0xff == 27:
        break

cv2.destroyAllWindows()
