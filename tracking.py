# -*- coding: utf-8 -*-
"""
Created on Tue Jul 14 10:16:41 2020

@author: mesut
"""


import cv2 
import numpy as np
import math
font = cv2.FONT_HERSHEY_SIMPLEX

cap = cv2.VideoCapture(0)

cap.set(3,320)
cap.set(4,240)

while True:
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    low_red = np.array([0,139,65])
    high_red = np.array([28,255,255])
    red_mask = cv2.inRange(hsv_frame,low_red,high_red)
    contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
    
    for cnt in contours:
        (x,y,w,h) = cv2.boundingRect(cnt)
        distancei = (2*3.14 * 180)/(w+h*360)*1000 + 3
        #print distancei
        distance = distancei *2.54
        distance = math.floor(distancei/2)
        cv2.rectangle(frame,(x,y),(x+w,y+h), (0,255,0),2)
        x_medium = int((x+x+w)/2)
        y_medium = int((y+y+h)/2)
        cv2.line(frame, (x_medium,0), (x_medium,480), (0,255,0),2)
        cv2.line(frame, (780,y_medium), (0,y_medium), (0,255,0),2)
        position = (3,30)
    
        cv2.putText(frame,'Distance = ' + str(distance) + ' cm', (3,50),font,0.5,(255,255,255),1)
        cv2.putText(
         frame, #numpy array on which text is written
         " x pos : "+str(x_medium)+" \n y pos : "+str(y_medium)+"", #text
         position, #position at which writing has to start
         cv2.FONT_HERSHEY_SIMPLEX, #font family
         0.5, #font size
         (255, 255, 0, 255),1)
        break
    
    
    
    
    cv2.imshow("Frame",frame)
    #cv2.imshow("", red_mask)
    
    
    
    key = cv2.waitKey(1)
    
    if key==27:
        break