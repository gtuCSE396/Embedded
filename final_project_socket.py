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
import sys
import socket
import thread
import threading

font = cv2.FONT_HERSHEY_SIMPLEX
setPointY=120 #it will get from desktop or mobile for y axes
setPointX=160 #it will get from desktop or mobile for y axes

setPointH=40
_pre_errorH=0

_pre_errorY=0
_pre_errorX=0
_integralY=0
_integralX=0
_integralH=0
period=200
periodH=200
myTime=0
ser=""
mode = 2
px = 0
py = 0
h = 0
motorAngels = ""
host = '192.168.137.61'
port = 9000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))
def video(cap):
   
    
    x_medium = 0
    y_medium = 0

    tmpX = 0
    tmpY = 0
    tmpD = 0
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
    #tmpX = tmpX - 160
    #tmpY = 120- tmpY 
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
    key = cv2.waitKey(1)   # print calculate(setPoint,y_medium,100,0.1,0.01,0.5)
    if key==27:
        sys.exit()
    #tmpX = tmpX - 160
    #tmpY = tmpY - 120
    return tmpX,tmpY,tmpD

def pid(tmpX,tmpY , kp , kd , ki , dt , _max , _min ):
    global myTime
    global setPointY#it will get from desktop or mobile for y axes
    global setPointX #it will get from desktop or mobile for y axes
    global _pre_errorY
    global _pre_errorX
    global _integralY
    global _integralX
    global period
    global ser
    global motorAngels
    
    
    millis = 0
    millis = int(round(time.time() * 1000))
    
    dy = 0
    dz = 0
   
    dt = 0.2
    _max = 13
    _min = -13
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
        temp = 200
        if(outputY < 0):
            dy = math.sqrt(-1*outputY/temp)
            dy = int(dy*-1)
        else:
            dy = int(math.sqrt(outputY/temp))
        
        if(outputX < 0):
            dx = math.sqrt(-1*outputX/temp)
            dx = int(dx *-1)
        else:
            dx = int(math.sqrt(outputX/temp))
        #dy =  int((outputY +3000) * (axisX) / (6000) + axisY)
        #dx =  int((outputX +3000) * (axisX) / (6000) + axisY)
        
        if(dy > _max):
            dy=_max
        if(dy < _min):
            dy=_min
        if(dx > _max):
            dx=_max
        if(dx < _min):
            dx=_min
            
        
        dz = -1*dy
        da = -1*dx
        print(str(dy)+"\n")
        print(str(dx)+"\n")
        ser.write(str(dy)+":"+str(da)+":"+str(dx)+":"+str(dz)+":4000\n")
        motorAngels = str(dy)+ " " + str(dz)+ " " + str(da) + " " + str(dx)
        
        #motorların adımları
        #dy ile dz (x ekseni) karşılıklı - da ile dx  (y ekseni) karşılıklı
def sektirme(x,y,h):
  


    #pid(x,y , 5 , 5 , 0 , 0.1 , 2 , -2 )
     
    global motorAngels
    
    global myTime
   
    global setPointH #it will get from desktop or mobile for y axes

    global _pre_errorH
    global _integralH
    global periodH
    global ser
    
    
    
    millis = 0
    millis = int(round(time.time() * 1000))
    
    dh=0
    ki= 0.1
    kp= 50
    kd= 300
    dt = 0.2
    _max = 13
    _min = -13
    if(millis > myTime + period):
        myTime= int(round(time.time() * 1000))
        
        errorH = setPointH - h
       
        PoutH= kp * errorH
       
        _integralH += errorH * ki
        IoutH = _integralH
   

        derivativeH = (errorH - _pre_errorH) / dt
        DoutH = kd * derivativeH
        
       

        # Calculate total output
        if(errorH > -15 or errorH < 15):
            IoutH = 0
        
        
        outputH = PoutH + IoutH +DoutH
      

        _pre_errorH = errorH
      
        
        print(str(outputH) +" :::: "+str(errorH)+"\n")
        
        temp = 200
        if(outputH < 0):
            dh = math.sqrt(-1*outputH/temp)
            dh = int(dh*-1)
        else:
            dh = int(math.sqrt(outputH/temp))
       
        #dy =  int((outputY +3000) * (axisX) / (6000) + axisY)
        #dx =  int((outputX +3000) * (axisX) / (6000) + axisY)
        
        if(dh > _max):
            dh=_max
        if(dh < _min):
            dh=_min
        
            
        
        dx=0
        dy=0
        print(str(dh)+"\n")
        if(x<100):
            dx=-2
        elif(x>220):
            dx=2
        else:
            dx=0
            
        if(y<100):
            dy=-2
        elif(y>220):
            dy=2
        else:
            dy=0
        ser.write(str(dh+dy)+":"+str(dh+dx)+":"+str(dh-dx)+":"+str(dh-dy)+":3000\n")
        
def recv(x):
    global mode
    data2 = ""
    
    while True:
        print("Receiver Started!")
        # server'dan gelen veriyi tutan degisken ------------
        data2 = sock.recv(1024).decode()
        
        #if not data2: sys.exit(0)
        if "DATAS" not in data2:
            if "has connected" not in data2:
                arr = data2.split()
                mode = str(arr[1])
                print(str(arr[0]) + ", " + str(arr[1]) + ", " + str(arr[2]) + "," + str(arr[3]))
                
             
        #time.sleep(.1)
                
availableFunction= threading.Semaphore(10)
def sendData(msg):
    print("senddata start")
    global px
    global py
    global h
    global motorAngels
    data = ""
    while True:
        
        print("senddata start")
        
        time.sleep(.1)

        
def main(msg):
    
    global ser
    global mode
    global px
    global py
    global h
    dh=0
    dx=3
    dy=6
    cap = cv2.VideoCapture(0)
    cap.set(3,320)
    cap.set(4,240)
    ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
    ser.flush()
    px = 0
    py = 0
    
    ki= 0.1
    kp= 30
    kd= 50
    dt = 0.2
    _max = 13
    _min = -13
    
    ready = 0
    counter = 0
    
    
    while True :
        print("main start")
        ##mode = get_mode()
        x , y , h = video(cap)
        if(x != 0):
            px = x
        if(y != 0):
            py = y
#         ser.flush()
#         print "hello"
#         ser.write("-1:0:0:1:1000")
#         time.sleep(2)
#         ser.write("0:0:0:0:1000")
#         time.sleep(2)
        #print(str(px)+"\t"+str(py)+"\t"+str(h)+"\n")
        if(mode == '1'): # 
            pid(px,py,kp,kd,ki,dt,_max,_min)
        elif(mode == '2'):
            if(ready == 0):
                pid(px,py,30,50,0.01,dt,_max,_min)
                if(px < setPointX+100 and px >= setPointX-100 and py < setPointY+100 and py >= setPointY-100):
                    if(counter == 200):
                        ser.write("0:0:0:0:10000\n")
                        #ser.write("-18:-18:-18:-18:10000\n")
                        ready = 1
                    counter = counter +1
                else:
                    counter = 0
            if(ready == 1):    
                sektirme(px,py,h)
        elif(mode == '3'):
            print ("not implemented \n")
        motorAngels = str(dh + dx)+ " " + str(dh - dx)+ " " + str(dh - dy) + " " + str(dh + dy)
        data =  "DATAS" + " " + str(px) + " " + str(py) + " " + str(h) + " " + motorAngels
        sockSender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sockSender.connect((host, port))    
        sockSender.send(data.encode())
        sockSender.close()
        
        

try:
    thread.start_new_thread(recv, (12, ))
    
    #kilitle
   # thread.start_new_thread(sendData, ('msg', ))
    #kilidi ac
    thread.start_new_thread(main, (10, ))
except:
    print("Error: unable to start thread")

while 1:
    pass
#if __name__== "__main__":
    #main()

    

