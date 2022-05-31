import cv2
import numpy as np
from matplotlib import pyplot as plt

cap = cv2.VideoCapture(1)

while(1):
    flag, image = cap.read() 
    HSV=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)

    def getpos(event,x,y,flags,param):
        if event==cv2.EVENT_LBUTTONDOWN:
            print(HSV[y,x])

    cv2.imshow('image',image)
#    cv2.imshow("imageHSV",HSV)

    cv2.setMouseCallback("image",getpos)
    cv2.waitKey(0)

