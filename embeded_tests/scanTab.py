import cv2
import numpy as np


def preProcessor(img):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 灰度
    blurred = cv2.bilateralFilter(gray, 2, 200, 200)  # 双边滤波降噪
    edged = cv2.Canny(blurred, 25, 200)  # 边缘识别
    # edged = cv2.dilate(edged, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀连接边缘
    cv2.imshow("edged", edged)
    contours, hierarchy = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)  # 寻找轮廓
    centers = []
    paperCnt = None
    if len(contours) > 0:

        # 按轮廓面积降序排列
        contours = sorted(contours, key=cv2.contourArea, reverse=True)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if 500 > area > 40:
                x, y, w, h = cv2.boundingRect(cnt)
                # cv2.rectangle(img,(x,y),(x+w,y+h),(0,255,0),2)
                # cv2.circle(img,(x+w//2,y+h//2),2,(0,0,255),3)
                # print(area/(w*h))
                if (0.6 * w * h <= area) & (0.87 * w * h >= area):  # 判定是否为圆
                    cv2.circle(img, (x + w // 2, y + h // 2), 2, (0, 0, 255), 3)
                    centers.append([x + w // 2, w + h // 2])

    return centers
    # cv2.polylines(img, [paperCnt], True, (0, 255, 0), 3)
    # return paperCnt
    # return [i[0] for i in paperCnt]


if __name__ == "__main__":
    # i2cy edit
    from cam_config import CAM

    cap = CAM

    # end edit
    # img = cv2.imread(".\Lib\doc_test.jpg")
    while (1):
        flag, frame = cap.read()
        print(preProcessor(frame))
        cv2.imshow("res", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
