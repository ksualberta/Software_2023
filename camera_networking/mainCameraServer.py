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


def get_message(connection:socket.socket,split_rate:int)->bytes:
    """
    Parameters: Connection socket and split rate\n
    Function: Calls socket.socket.recv() 'split rate' number of times\n\tand aggregates the message\n
    Affects:Nothing\n
    Returns: Returns the message in bytes
    """

    returnMessage = b''

    i = 0
    while i < split_rate:
        try:
            msg_length = connection.recv(HEADER).decode(FORMAT)
            msg_length = int(msg_length)

            if msg_length:
                msg = b''

                while len(msg) < msg_length:
                    msg += connection.recv(msg_length - len(msg))
            
                returnMessage += msg
                i+=1
        except:
            print("Error")

    return returnMessage


##-----------------------------------------------------------------------------------------#
## HANDEL CLIENT - Intakes connection and its adrress
def handle_client(conn:socket.socket , addr):
    print(f'[SERVER] NEW CONNECTION AT {addr}')
    connected = True ## False to end conn

    while connected:
        split_msg_length = conn.recv(HEADER).decode(FORMAT)
        split_msg = conn.recv(int(split_msg_length)).decode(FORMAT)
        split_msg = int(split_msg)

        if split_msg:
            msg = get_message(conn,split_msg)
            try:
                msg = pickle.loads(msg)
                print(len(msg))
                msg_type = type(msg)
                if msg_type ==  str :
                    if msg == DISMES:
                        print(f'[CLIENT {addr}] DISCONNECTING')
                        connected = False
                    else:
                        print(f'[CLIENT {addr}] {msg}')
                elif msg_type == bytes:
                    msg = np.frombuffer(msg,np.byte)
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
            except:
                ok = 1
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'): ## if q is pressed disconnect
                connected = False
        #msg_one_len = conn.recv(HEADER).decode(FORMAT) ## Checks Header for message length
        #if msg_one_len: ## Check for no data
         #   msg_one_len = int(msg_one_len) 
          #  msg_one = b'' ## msg is the actual data being transfered
           # while len(msg_one) < msg_one_len: ## Collects all data to msg
            #    msg_temp = conn.recv(msg_one_len-len(msg_one)) ## ensures all data collected
             #   msg_one += msg_temp

       # msg_two_len = conn.recv(HEADER).decode(FORMAT)

        #if msg_two_len:
         #   msg_two_len = int(msg_two_len) 
          #  msg_two = b'' ## msg is the actual data being transfered
           # while len(msg_two) < msg_two_len: ## Collects all data to msg
            #    msg_temp = conn.recv(msg_two_len-len(msg_two)) ## ensures all data collected
             #   msg_two += msg_temp  

      #  if msg_one_len and msg_two_len:
       #     msg = msg_one + msg_two          
        #    msg = pickle.loads(msg) ## Collected Message Unloaded
         #   msg_type = type(msg)
            
          #  print(type(msg))
            

           # if msg_type ==  str : ## if string show in console, or Disconnecting
            #    if msg == DISMES:
             #       print(f'[CLIENT {addr}] DISCONNECTING')
              #      connected = False
               # else: 
                #    print(f'[CLIENT {addr}] {msg}')

            #elif msg_type == bytes: ## if list turn into and show image
            #else:    
                #print("made it here")

                #msg = np.frombuffer(msg,np.byte) ## Byte Repair
                #msg = cv2.imdecode(msg, 1)
                #aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
                #parameters = aruco.DetectorParameters()
                #detector = aruco.ArucoDetector(aruco_dict,parameters)
                #corners, ids, rejectedImgPoints = detector.detectMarkers(image=msg)

                #if type(ids) == type(None):
                   #cv2.imshow("RECIVEDVIDEO", msg)
                #else:
                   #for id in ids:
                        #print("[DETECED MARKER]: {}\n".format(id))
                        #editedFrame = aruco.drawDetectedMarkers(image=msg.copy(),corners=corners,ids=ids)
                        #cv2.imshow("DETECTED IMAGE",editedFrame)

                #key = cv2.waitKey(1) & 0xFF
                #if key == ord('q'): ## if q is pressed disconnect
                    #connected = False
                
    conn.close()
    cv2.destroyAllWindows()
    print(f"[SERVER] CLIENT {addr} DISCONNECTED")

start()

            

