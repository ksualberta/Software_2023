import socket
import cv2
import pickle
import numpy as np
import time
#import matplotlib

##-----------------------------------------------------------------------------------------#
## CONSTANT VALUES
PORT   = 7505
#SERVER = "192.168.1.2" ## ez adress switch
SERVER = socket.gethostbyname(socket.gethostname())
THREAD = 1
ADDR   = (SERVER , PORT) ## basic informaton for contacting server
HEADER = 16 ## How big the header is on the incoming info
FORMAT = 'utf-8' ## Format of the bytes used
DISMISS = '!END' ## Message to disconnect from server
JPEGQUALITY = 25 ## Quality of image outgoing 0-100
ENCODEPARAM = [int(cv2.IMWRITE_JPEG_QUALITY), JPEGQUALITY]

SPLIT_RATE = 1

SD  = (480  , 640 )
HD  = (720  , 1280)
FHD = (1080 , 1920) ## STANDARD MONITOR
QHD = (1440 , 2560) ## NOT WORK
UHD = (2160 , 3840) ## NOT WORK

REZ = SD

CAMID = 0 ## ID of camera, depends on how many devices you have

def send_EOS(connection:socket.socket):
    """
    Message that will send end of stream message to server\n
    If message is unable to send, meaning connection is broken\n
    In this case, will simply shut down
    """
    dismiss_msg = DISMISS.encode('utf-8')
    dismiss_msg_length = str(len(dismiss_msg)).encode('utf-8')
    dismiss_msg_length += b' ' * (HEADER - len(dismiss_msg_length))

    try:
        connection.send(dismiss_msg_length)
        connection.send(dismiss_msg)
    except Exception:
        print("Couldn't send a message: {message}\n".format(message = Exception))




##-----------------------------------------------------------------------------------------#
## START
def start():
    """
    Starting point for program\n
    Can throw an except if the client doesn't connect or looses connection\n
    Can throw an except if reading a camera frame doesn't work\n
    Calls send_EOS() to handle exceptions\n
    """
    print('[CLIENT] STARTING UP')
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(ADDR)
    print(f'[CLIENT] CONNECTED TO {SERVER}, {PORT}')
    camera = cam_set(CAMID, REZ, client)
    try:
        video_send(camera , client)
    except Exception as ext:
        print('[CLIENT] ERROR, DISCONNECTING')
        send_EOS(client)
        print(ext)
        #sendData(client, 'ERROR OCURRED')
        #sendData(client, DISMES)
        client.close()



##-----------------------------------------------------------------------------------------#
## CAM SET - Intakes Camera ID and Tuples rez (y,x), returns camera for cv2
def cam_set(camID, rez, client):
    print(f'[CLIENT] Linking Camera #{camID}')
    #sendData(client, (f'Linking Camera #{camID}'))
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
        ret, frame = camera.read()
        if ret == True:
            frame = cv2.imencode('.jpg', frame, ENCODEPARAM)[1].tobytes()
            #frame =  frame.tobytes()
            split_data(client, frame, SPLIT_RATE)           


##-----------------------------------------------------------------------------------------#
## SEND - Intakes data and sends to server
def split_data(client:socket.socket, msg:bytes, split_rate:int)-> None:
    """
    Parameters: Client, message, and split rate\n
    Function: Splits the messages into 'split rate' number of messages and message lengths\n
    Affects: Affects the client by internalling calling send_data() method\n
    Returns: Nothing
    """
    

    main_message = pickle.dumps(msg)
    main_message_length = len(main_message)

    
    msg_length = int(main_message_length/split_rate)

    msg_list = list()
    msg_len_list = list()

    i = 1

    while i <= split_rate:
        lower_bound = (i - 1) * msg_length
        if i != split_rate:
            upper_bound = i * msg_length

            temp_msg = main_message[lower_bound:upper_bound]
        else:

            temp_msg = main_message[lower_bound:]
            

        temp_len = str(len(temp_msg)).encode(FORMAT)
        temp_len += b' ' * (HEADER - len(temp_len))
        msg_list.append(temp_msg)
        msg_len_list.append(temp_len)
        i+=1

    send_data(client,msg_list,msg_len_list)

def send_data(client:socket.socket,msg_list:list,msg_len_list:list):
    """
    Recieves the client, the split messages, and the split messages' lengths\n
    First, send the thread Id
    Then send the split rate to the server\n
    Then sends each len and message tuple to the server\n
    """
    i = 0

    thread_message = str(THREAD).encode(FORMAT)
    thread_message_len = str(len(thread_message)).encode(FORMAT)
    thread_message_len += b' ' * (HEADER - len(thread_message_len))
    client.send(thread_message_len)
    client.send(thread_message)

    split_rate_msg = str(len(msg_list)).encode(FORMAT)
    split_rate_msg_len = str(len(split_rate_msg)).encode(FORMAT)
    split_rate_msg_len += b' ' * (HEADER - len(split_rate_msg_len))

    client.send(split_rate_msg_len)
    client.send(split_rate_msg)

    
    while i < len(msg_list):
        client.send(msg_len_list[i])
        client.send(msg_list[i])
        i+=1



start()
