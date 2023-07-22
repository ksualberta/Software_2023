import socket
import threading
import cv2.aruco as aruco
import cv2
import pickle
import numpy as np
import os
#import matplotlib

##-----------------------------------------------------------------------------------------#
## CONSTANT VALUES
PORT   = 7505
SERVER = "0.0.0.0" 
ADDR   = (SERVER , PORT) ## basic informaton for contacting server
HEADER = 16 ## How big the header is on the incoming info
FORMAT = 'utf-8' ## Format of the bytes used
DISMES = '!END' ## Message to disconnect from server

##-----------------------------------------------------------------------------------------#
## START
def start():
    #os.environ["QT_QPA_PLATFORM"] = "xcb"
    print('[SERVER] STARTING UP')
    host = socket.socket(socket.AF_INET , socket.SOCK_STREAM) ## Creates stream type server
    host.bind(ADDR) ## Binds Server to adress
    host.listen(5) ## Server listening for connections with buffer 5
    print('[SERVER] STARTUP COMPLETE')
    print(f'[SERVER] LISTENING ON {SERVER}, {PORT}')
    main_run(host)

##-----------------------------------------------------------------------------------------#
## MAIN RUN - Intakes Host Socket
def main_run(host):
    while True:
        conn , addr = host.accept() # Accepts and stores incoming conneciton
        thread = threading.Thread(target = handle_client, args= (conn, addr)) 
        thread.start() # Puts each client on own thread
        print(f'[SERVER] NEW CONNECTION : {threading.active_count()-1} ACTIVE')

##-----------------------------------------------------------------------------------------#
## HANDEL CLIENT - Intakes connection and its adrress
def handle_client(conn , addr):
    print(f'[SERVER] NEW CONNECTION AT {addr}')
    connected = True ## False to end conn

    while connected:
        msg_one_len = conn.recv(HEADER).decode(FORMAT) ## Checks Header for message length
        if msg_one_len: ## Check for no data
            msg_one_len = int(msg_one_len) 
            msg_one = b'' ## msg is the actual data being transfered
            while len(msg_one) < msg_one_len: ## Collects all data to msg
                msg_temp = conn.recv(msg_one_len-len(msg_one)) ## ensures all data collected
                msg_one += msg_temp

        msg_two_len = conn.recv(HEADER).decode(FORMAT)

        if msg_two_len:
            msg_two_len = int(msg_two_len) 
            msg_two = b'' ## msg is the actual data being transfered
            while len(msg_two) < msg_two_len: ## Collects all data to msg
                msg_temp = conn.recv(msg_two_len-len(msg_two)) ## ensures all data collected
                msg_two += msg_temp  

        if msg_one_len and msg_two_len:
            msg = msg_one + msg_two          
            msg = pickle.loads(msg) ## Collected Message Unloaded
            msg_type = type(msg)
            
            print(type(msg))
            

            if msg_type ==  str : ## if string show in console, or Disconnecting
                if msg == DISMES:
                    print(f'[CLIENT {addr}] DISCONNECTING')
                    connected = False
                else: 
                    print(f'[CLIENT {addr}] {msg}')

            #elif msg_type == bytes: ## if list turn into and show image
            else:    
                #print("made it here")

                msg = np.frombuffer(msg,np.byte) ## Byte Repair
                msg = cv2.imdecode(msg, 1)
                aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
                parameters = aruco.DetectorParameters()
                detector = aruco.ArucoDetector(aruco_dict,parameters)
                corners, ids, rejectedImgPoints = detector.detectMarkers(image=msg)

                if type(ids) == type(None):
                    cv2.imshow("RECIVEDVIDEO", msg)
                else:
                    for id in ids:
                        print("[DETECED MARKER]: {}\n".format(id))
                    editedFrame = aruco.drawDetectedMarkers(image=msg.copy(),corners=corners,ids=ids)
                    cv2.imshow("DETECTED IMAGE",editedFrame)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'): ## if q is pressed disconnect
                    connected = False
                
    conn.close()
    cv2.destroyAllWindows()
    print(f"[SERVER] CLIENT {addr} DISCONNECTED")

start()

            

