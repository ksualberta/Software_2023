import socket
import threading
import pickle
import numpy as np
import cv2

FORMAT = "utf-8"
HEADER = 16
PORT = 5051
SERVER = '0.0.0.0'
ADDR = (SERVER,PORT)

def handle_client(connection:socket.socket,addr):
    print("[CONNECTION MADE] {}".format(addr))
    
    while True:
        messageLength = int(connection.recv(HEADER).decode('utf-8'))
        if messageLength:
            message = b''
            print("\nMade it here 1")
            while len(message) < messageLength:
                temp_msg = connection.recv(messageLength - len(message))
                message += temp_msg

            print("\nMade it here 2")
            message = pickle.loads(message)
            messageType = type(messageLength)

            messageNumpy = np.frombuffer(buffer=message,dtype=np.byte)
            decodedNumpy = cv2.imdecode(buf=messageNumpy,flags=1)
            print("\nMade it here")
            cv2.imshow("Arm_cam",decodedNumpy)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

def start_server():
    server = socket.socket(family=socket.AF_INET,type=socket.SOCK_STREAM)
    server.bind(ADDR)

    server.listen()

    print("[LISTENING ON SERVER] {}".format(socket.gethostbyname(socket.gethostname())))

    while True:
        connection, addr = server.accept()
        thread = threading.Thread(target=handle_client,args=[connection,addr])
        thread.start()

start_server()