import socket
import _thread
import time

host = '127.0.0.1'
port = 9000

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((host, port))

def recv(x):
    data2 = ""
    print(x)
    while True:
        # server'dan gelen veriyi tutan degisken ------------
        data2 = sock.recv(1024).decode()  
        if not data2: sys.exit(0)
        if "embedded" not in data2:
            if "Client has connected" not in data2: 
                print(data2)

def sendData(msg):
    data = "embedded: "
    counter = 1
    while 1:
        sockSender = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sockSender.connect((host, port))
        # server'a veri gönderen değisken ----------
        data += str(counter)
        counter += 1
        # data'nın yollandıgı satir -----------
        sockSender.send(data.encode())
        sockSender.close()   
        time.sleep(.01)

try:
    _thread.start_new_thread(recv, (12, ))
    _thread.start_new_thread(sendData, ('msg', ))
except:
    print("Error: unable to start thread")

while 1:
    pass

sock.close()
