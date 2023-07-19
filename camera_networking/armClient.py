import socket
import pickle
import cv2

FORMAT = "utf-8"
HEADER = 16
PORT = 5051
SERVER = socket.gethostbyname(socket.gethostname())
ADDR = (SERVER,PORT)
JPEGQUALITY = 25
ENCODEPARAM = [int(cv2.IMWRITE_JPEG_QUALITY), JPEGQUALITY]

def sendData(client , msg):
    msg = pickle.dumps(msg)
    msg_len = str(len(msg)).encode(FORMAT)
    msg_len += b' ' * (HEADER - len(msg_len))
    client.send(msg_len)
    client.send(msg)
    print("SEND DATA")

def start_client():
    camera = cv2.VideoCapture(1)
    client = socket.socket(family=socket.AF_INET,type=socket.SOCK_STREAM)
    client.connect(ADDR)


    while True:
        ret, frame = camera.read()
        frame = cv2.imencode('.jpg', frame, ENCODEPARAM)[1].tobytes()
        sendData(client=client,msg=frame)

start_client()