# -*- coding: utf-8 -*-
"""
Created on Tue Jul 14 10:16:41 2020
@author: mesut
"""




import cv2
import numpy as np
import math
import time
import serial

_pre_errorY=0
_pre_errorX=0
_integralY=0
_integralX=0
period=200
myTime=0
#y axes is pv get from camera - current position
#dt is time interval
# def calculate(setpoint,pv,_Kp,_Ki,_Kd,_dt):
#     error = setpoint - pv
#     Pout = _Kp * error
#     _integral += error * _dt
#     Iout = _Ki *_integral
# 
#     derivative = (error - _pre_error) / _dt
#     Dout = _Kd * derivative
# 
#     # Calculate total output
#     output = Pout + Iout + Dout 
# 
#     _pre_error = error
# 
#     return output

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
ser.flush()

font = cv2.FONT_HERSHEY_SIMPLEX

setPointY=120 #it will get from desktop or mobile for y axes
setPointX=120 #it will get from desktop or mobile for y axes

cap = cv2.VideoCapture(0)

cap.set(3,320)
cap.set(4,240)
x_medium = 0
y_medium = 0

tmpX = 0
tmpY = 0
tmpD = 0
while True:
   
    
    _, frame = cap.read()
    hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
      
    sensitivity = 15
    low_red = np.array([2,85,51])
    high_red = np.array([33,223,253])  
      
      
    red_mask = cv2.inRange(hsv_frame,low_red,high_red)
    _ ,contours, _= cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=lambda x:cv2.contourArea(x), reverse=True)
    #x_medium = 0
    #y_medium = 0
    for cnt in contours:
        (x,y,w,h) = cv2.boundingRect(cnt)
        distancei = (2*3.14 * 180)/(w+h*360)*1000 + 3
        #print distancei
        distance = distancei *2.54
        distance = math.floor(distancei/2)
        #cv2.rectangle(frame,(x,y),(x+w,y+h), (0,255,0),2)
        x_medium = int((x+x+w)/2)
        y_medium = int((y+y+h)/2)
        cv2.line(frame, (x_medium,0), (x_medium,480), (0,255,0),1)
        cv2.line(frame, (780,y_medium), (0,y_medium), (0,255,0),1)
        
        
        tmpX=x_medium
        tmpY=y_medium
        tmpD=distance
        
        break
    
    position = (3,30)
    
    cv2.putText(frame,'Distance = ' + str(tmpD) + ' cm', (3,50),font,0.5,(255,255,255),1)
    cv2.putText(
     frame, #numpy array on which text is written
     " x pos : "+str(tmpX)+" \n y pos : "+str(tmpY)+"", #text
     position, #position at which writing has to start
     cv2.FONT_HERSHEY_SIMPLEX, #font family
     0.5, #font size
     (255, 255, 0, 255),1)
    cv2.imshow("Frame",frame)
    #cv2.imshow("", red_mask)
    
    
    
    key = cv2.waitKey(1)
    
   # print calculate(setPoint,y_medium,100,0.1,0.01,0.5)
    millis = 0
    millis = int(round(time.time() * 1000))
    
    dy = 0
    dz = 0
    ki= 0.1
    kp= 30
    kd= 50
    dt = 0.2
    if(millis > myTime + period):
        myTime= int(round(time.time() * 1000))
        errorY = setPointY - tmpY
        errorX = setPointX - tmpX
        PoutY = kp * errorY
        PoutX= kp * errorX
        _integralY += errorY * ki
        _integralX += errorX * ki
        IoutY = _integralY
        IoutX = _integralX

        derivativeY = (errorY - _pre_errorY) / dt
        DoutY = kd * derivativeY
        
        derivativeX = (errorX - _pre_errorX) / dt
        DoutX = kd * derivativeX

        # Calculate total output
        if(errorY > -15 or errorY < 15):
            IoutY = 0
        if(errorX > -15 or errorX < 15):
            IoutX = 0
        
        outputY = PoutY + IoutY +DoutY
        outputX = PoutX + IoutX +DoutX

        _pre_errorY = errorY
        _pre_errorX = errorX
        axisX=10
        axisY=-5
        print(str(outputY) +" :::: "+str(errorY)+"\n")
        print(str(outputX) +" :::: "+str(errorX)+"\n")
        dy =  int((outputY +5000) * (axisX) / (10000) + axisY)
        dx =  int((outputX +5000) * (axisX) / (10000) + axisY)
        if(dy > 7):
            dy=7
        if(dy < -7):
            dy=-7
        if(dx > 7):
            dx=7
        if(dx < -7):
            dx=-7
        
        dz = -1*dy
        da = -1*dx
        print(str(dy)+"\n")
        print(str(dx)+"\n")
        
        ser.write(str(dy)+":"+str(da)+":"+str(dx)+":"+str(dz)+":5000\n")
        
        #ser.write(str(da)+":0:0:"+str(dx)+":5000\n")
        
       
    
    if key==27:
        break


