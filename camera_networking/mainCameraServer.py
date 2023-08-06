import socket
import threading
import cv2.aruco as aruco
import cv2
import pickle
import numpy as np
import os
from tkinter import *
from PIL import ImageTk,Image



PORT   = 7505
SERVER = "0.0.0.0" 
ADDR   = (SERVER , PORT) ## basic informaton for contacting server
HEADER = 16 ## How big the header is on the incoming info
FORMAT = 'utf-8' ## Format of the bytes used
DISMES = '!END' ## Message to disconnect from server


def start():
    """
    Creates the main window and creates another thread that runs inside of the mainloop
    """
    os.environ["QT_QPA_PLATFORM"] = "xcb"
    print('[SERVER] STARTING UP')
    host = socket.socket(socket.AF_INET , socket.SOCK_STREAM) ## Creates stream type server
    host.bind(ADDR) ## Binds Server to adress
    host.listen(5) ## Server listening for connections with buffer 5
    print('[SERVER] STARTUP COMPLETE')
    print(f'[SERVER] LISTENING ON {SERVER}, {PORT}')
    label_tuple = create_main_window()
    main_window = label_tuple[3]

    newThread = threading.Thread(target=main_run,args=[host,label_tuple,main_window])
    newThread.daemon = True
    newThread.start()
    main_window.mainloop()
    


def main_run(host,label_tuple,main_window:Tk):
    while True:
        conn , addr = host.accept() # Accepts and stores incoming conneciton
        thread = threading.Thread(target = handle_client, args= (conn, addr, label_tuple,main_window))
        thread.daemon = True #need this  
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


def create_main_window()->tuple:
    """
    Creates main window and a frame inside of that window\n
    Creates three labels that in frame:\n
    1. main-cam, 2. aruco-detected, 3. hand-camera\n
    Returns labels as such: Tuple(main-cam, aruco-detected, hand-camera)
    """
    rez = (1280,720)

    #create the main window in tkinter
    mainWindow = Tk()
    mainWindow.geometry(newGeometry="{}x{}".format(rez[0] * 2, rez[1] * 2))
    mainWindow.title("SPEAR MAIN FEED")

    pil_image = Image.open("Software_2023/camera_networking/placeholder.png")
    image = ImageTk.PhotoImage(image=pil_image)

    #create the three labels that will be controlled
    mainFeedLabel = Label(image=image,master=mainWindow,width=rez[0],height=rez[1],highlightbackground='black',highlightthickness=2)
    arucoFeedLabel = Label(image=image,master=mainWindow,width=rez[0],height=rez[1],highlightbackground='black',highlightthickness=2)
    handCameraLabel = Label(image=image,master=mainWindow,width=rez[0],height=rez[1],highlightbackground='black',highlightthickness=2)
    blankCameraLabel = Label(image=image,master=mainWindow,width=rez[0],height=rez[1],highlightbackground='black',highlightthickness=2)

    #place the labels on the main window using grid

    mainFeedLabel.grid(row=0,column=0)
    arucoFeedLabel.grid(row=0,column=1)
    handCameraLabel.grid(row=1,column=0)
    blankCameraLabel.grid(row=1,column=1)
    #mainWindow.mainloop()

    return (mainFeedLabel,arucoFeedLabel,handCameraLabel,mainWindow,image)


def update_label(label:Label,img:np.ndarray):
    """
    Takes in the label to update and the image to update with\n
    Sets the label using the PIL library
    Updates the frame
    """
    rgb_image = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    pil_image = Image.fromarray(rgb_image)
    image = ImageTk.PhotoImage(image=pil_image)
    label.config(image=image)
    label.image = image

def update_label_in_error(label:Label):
    """
    Method only gets called if there is an exception
    Resets the label arguments to the SPEAR logo
    """
    spear_logo = Image.open("Software_2023/camera_networking/placeholder.png")
    image = ImageTk.PhotoImage(image=spear_logo)
    label.config(image=image)
    label.image = image


def handle_client(conn:socket.socket , addr, label_tuple:tuple,main_window:Tk):
    """
    Takes the new connection and it's IP address.\n
    Uses this send each connection to it's respective frame in the Tkinter frame
    Updates the Tkingter frames in the respective frames
    """
    print(f'[SERVER] NEW CONNECTION AT {addr}')
    connected = True ## False to end conn

    while connected:

        thread_msg_length = conn.recv(HEADER).decode(FORMAT)
        thread_msg = conn.recv(int(thread_msg_length)).decode(FORMAT)
        thread_msg = int(thread_msg)       

        split_msg_length = conn.recv(HEADER).decode(FORMAT)
        split_msg = conn.recv(int(split_msg_length)).decode(FORMAT)
        split_msg = int(split_msg)

        if split_msg:
            msg = get_message(conn,split_msg)
            try:
                msg = pickle.loads(msg)

                msg_type = type(msg)
                if msg_type ==  str :
                    if msg == DISMES:
                        if thread_msg == 1:
                            update_label_in_error(label_tuple[0])
                            update_label_in_error(label_tuple[1])
                        elif thread_msg == 2:
                            update_label_in_error(label_tuple[2])
                        print(f'[CLIENT {addr}] DISCONNECTING')
                        connected = False
                    else:
                        print(f'[CLIENT {addr}] {msg}')
                elif msg_type == bytes:
                    msg = np.frombuffer(msg,np.byte)
                    msg = cv2.imdecode(msg, 1)

                    if thread_msg == 1:
                        aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
                        parameters = aruco.DetectorParameters()
                        detector = aruco.ArucoDetector(aruco_dict,parameters)
                        corners, ids, rejectedImgPoints = detector.detectMarkers(image=msg)

                        if type(ids) == type(None):
                            update_label(label_tuple[0],msg)
                        else:
                            for id in ids:
                                print("[DETECED MARKER]: {}\n".format(id))
                                editedFrame = aruco.drawDetectedMarkers(image=msg.copy(),corners=corners,ids=ids)
                                update_label(label_tuple[1],editedFrame)
                    
                    elif thread_msg == 2:
                        update_label(label_tuple[2],msg)
                    main_window.update()
            except:
                if thread_msg == 1:
                    update_label_in_error(label_tuple[0])
                    update_label_in_error(label_tuple[1])
                elif thread_msg == 2:
                    update_label_in_error(label_tuple[2])
                connected = False
    conn.close()
    cv2.destroyAllWindows()
    print(f"[SERVER] CLIENT {addr} DISCONNECTED")

start()

            

