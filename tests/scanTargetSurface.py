import cv2
import numpy as np


def preProcessor(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 灰度
    blurred = cv2.bilateralFilter(gray, 2, 200, 200)  # 双边滤波降噪
    edged = cv2.Canny(blurred, 25, 200)  # 边缘识别
    #edged = cv2.dilate(edged, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀连接边缘
    cv2.imshow("mask",edged)
    contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 寻找轮廓

    paperCnt = None
    if len(contours) > 0:
        # 按轮廓面积降序排列
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        for c in contours:
            # 近似轮廓
            if cv2.contourArea(c) > 5000 :
                cv2.drawContours(img,c,-1,(255,0,0),2)
                peri = cv2.arcLength(c, True)
                approx = cv2.approxPolyDP(c, 0.1 * peri, True)
                if (len(approx) == 4)&(not cv2.isContourConvex(c)):
                    cv2.drawContours(img,c,-1,(0,255,0),2)
                    #x,y,w,h = cv2.minAreaRect(c)
                    #cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                    rect = cv2.minAreaRect(c)
                    box = cv2.boxPoints(rect)
                    box = np.int0(box)
                    cv2.drawContours(img,[box],0,(0,0,255),2)


                    paperCnt = approx
                    break
    # cv2.polylines(img, [paperCnt], True, (0, 255, 0), 3)
    return paperCnt
    # return [i[0] for i in paperCnt]



if __name__ == "__main__":
    cap = cv2.VideoCapture(1)
    # img = cv2.imread(".\Lib\doc_test.jpg")
    while(1):

        flag,frame = cap.read()
        preProcessor(frame)
        cv2.imshow("res",frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
