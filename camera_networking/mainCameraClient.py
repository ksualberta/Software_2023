import socket
import cv2
import pickle
import numpy as np
#import matplotlib

##-----------------------------------------------------------------------------------------#
## CONSTANT VALUES
PORT   = 7505
#SERVER = "192.168.1.2" ## ez adress switch
SERVER = socket.gethostbyname(socket.gethostname())
ADDR   = (SERVER , PORT) ## basic informaton for contacting server
HEADER = 16 ## How big the header is on the incoming info
FORMAT = 'utf-8' ## Format of the bytes used
DISMES = '!END' ## Message to disconnect from server
JPEGQUALITY = 100 ## Quality of image outgoing 0-100
ENCODEPARAM = [int(cv2.IMWRITE_JPEG_QUALITY), JPEGQUALITY]

SD  = (480  , 640 )
HD  = (720  , 1280)
FHD = (1080 , 1920) ## STANDARD MONITOR
QHD = (1440 , 2560) ## NOT WORK
UHD = (2160 , 3840) ## NOT WORK

REZ = FHD

CAMID = 0 ## ID of camera, depends on how many devices you have

##-----------------------------------------------------------------------------------------#
## START
def start():
    print('[CLIENT] STARTING UP')
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(ADDR)
    print(f'[CLIENT] CONNECTED TO {SERVER}, {PORT}')
    camera = cam_set(CAMID, REZ, client)
    try:
        video_send(camera , client)
    except Exception as ext:
        print('[CLIENT] ERROR, DISCONNECTING')
        print(ext)
        sendData(client, 'ERROR OCURRED')
        sendData(client, DISMES)
        client.close()



##-----------------------------------------------------------------------------------------#
## CAM SET - Intakes Camera ID and Tuples rez (y,x), returns camera for cv2
def cam_set(camID, rez, client):
    print(f'[CLIENT] Linking Camera #{camID}')
    sendData(client, (f'Linking Camera #{camID}'))
    camera = cv2.VideoCapture(camID)
    camera.set(cv2.CAP_PROP_FRAME_HEIGHT, rez[0])
    camera.set(cv2.CAP_PROP_FRAME_WIDTH, rez[1])
    camera.set(cv2.CAP_PROP_BUFFERSIZE, 2)
    print(f'[CLIENT] Camera #{camID} set at {rez[0]} x {rez[1]}')
    return camera

##-----------------------------------------------------------------------------------------#
## VIDEOSEND - Intakes a camera and a server connection
def video_send(camera , client):
    while camera.isOpened():
        img, frame = camera.read()
        if img == True:
           # frame = cv2.imencode('.jpg', frame, ENCODEPARAM)[1].tobytes()
            frame =  frame.tobytes()
            sendData(client, frame)           


##-----------------------------------------------------------------------------------------#
## SEND - Intakes data and sends to server
def sendData(client , msg):
    msg = pickle.dumps(msg)
    msg_len = str(len(msg)).encode(FORMAT)
    msg_len += b' ' * (HEADER - len(msg_len))
    client.send(msg_len)
    client.send(msg)



start()
