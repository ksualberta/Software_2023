import socket
import cv2
import pickle
import numpy as np
import mss
#import matplotlib

##-----------------------------------------------------------------------------------------#
## CONSTANT VALUES
PORT   = 7505
SERVER = socket.gethostbyname(socket.gethostname()) ## ez adress switch
#SERVER = socket.gethostbyname(socket.gethostname())
ADDR   = (SERVER , PORT) ## basic informaton for contacting server
HEADER = 16 ## How big the header is on the incoming info
FORMAT = 'utf-8' ## Format of the bytes used
DISMES = '!END' ## Message to disconnect from server
JPEGQUALITY = 25 ## Quality of image outgoing 0-100
ENCODEPARAM = [int(cv2.IMWRITE_JPEG_QUALITY), JPEGQUALITY]

SD  = (480  , 640 )
HD  = (720  , 1280)
FHD = (1080 , 1920) ## STANDARD MONITOR
QHD = (1440 , 2560) ## NOT WORK
UHD = (2160 , 3840) ## NOT WORK

REZ = FHD

CAMID = 0 ## ID of camera, depends on how many devices you have




def start():
    print('[CLIENT] STARTING UP')
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(ADDR)
    print(f'[CLIENT] CONNECTED TO {SERVER}, {PORT}')

    with mss.mss() as sct:
        monitor = {"top": 0, "left": 0, "width": REZ[1], "height": REZ[0]}

        while True:
            frame = np.array(sct.grab(monitor))
            frame = frame.tobytes()
            sendData(client,frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

def sendData(client , msg):
    msg = pickle.dumps(msg)
    msg_len = str(len(msg)).encode(FORMAT)
    msg_len += b' ' * (HEADER - len(msg_len))
    client.send(msg_len)
    client.send(msg)

    


start()
