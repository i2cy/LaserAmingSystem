import time

import cv2

# from matplotlib import pyplot as plt

cap = cv2.VideoCapture(
    'rkisp device=/dev/video1 io-mode=4 ! video/x-raw,format=NV12,width=640,height=480,framerate=120/1 ! videoconvert '
    '! appsink',
    cv2.CAP_GSTREAMER)  # VideoCapture对象它的参数可以是设备索引或者一个视频文件名

cnt = 0
t0 = time.time()
while True:
    ret, frame = cap.read()  # 一帧一帧捕捉
    # blur = cv2.bilateralFilter(frame, 9, 75, 75)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 我们对帧的操作
    ret, mask = cv2.threshold(gray, 210, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 寻找轮廓
    # cv2.imshow("mask",mask)
    if len(contours) is 0:
        continue
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    cnt = contours[0]
    # cv2.drawContours(frame, contours, 0, (0,255,0), 3)
    # print(cv2.contourArea(cnt))
    x, y, w, h = cv2.boundingRect(cnt)
    if 100 < w * h < 600:  # &(w*h<600):

        # cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
        # cv2.circle(frame, (x + w // 2, y + h // 2), 2, (0, 255, 0), 3)
        print([x + w // 2, y + h // 2])

    print("FPS: {}".format(1 / (time.time() - t0)))

    t0 = time.time()

    # cv2.imshow('frame', frame)  # 显示返回的每帧
    # if cv2.waitKey(1) & 0xFF == ord('q'):
    #    break
cap.release()  # 当所有事完成，释放 VideoCapture 对象
cv2.destroyAllWindows()
