# -*- coding: utf-8 -*-
import socket
import thread
import time

host = '192.168.137.61'
port = 9000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))

def recv(x):
    data2 = ""
    print(x)
    while True:
        # server'dan gelen veriyi tutan degisken ------------
        data2 = sock.recv(1024).decode()
        #if not data2: sys.exit(0)
        if "embedded" not in data2:
            if "has connected" not in data2:
                arr = data2.split()
                print(str(arr[0]) + ", " + str(arr[1]) + ", " + str(arr[2]) + ", " + str(arr[3]))
                

#bu fonksiyon desktop 
def sendData(msg):
    data = "embedded: "
    counter = 1
    while True:
        sockSender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sockSender.connect((host, port))
       
        data += str(counter)
        counter += 1
    
        sockSender.send(data.encode())
        sockSender.close()
        time.sleep(.1) #0.01 saniyede veri geliyor

try:
    thread.start_new_thread(recv, (12, ))
    #thread.start_new_thread(sendData, ('msg', ))
except:
    print("Error: unable to start thread")

while 1:
    pass

sock.close()
