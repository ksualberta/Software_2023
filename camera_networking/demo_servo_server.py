import socket
import threading
import subprocess
import numpy as np
import cv2
import pickle
import time

PORT = 5050
HEADER = 64 #the size of the buffer that declares the size of the buffer for the incoming message
SERVER = "0.0.0.0"
INTERNAL_SERVER = socket.gethostbyname(socket.gethostname())
INTERNAL_PORT = 9050
EXTERNAL_SERVER = "192.168.1.1"
EXTERNAL_PORT = 9050
ADDR = (SERVER,PORT)
INTERNAL_ADDR = (INTERNAL_SERVER,INTERNAL_PORT)
EXTERNAL_ADDR = (EXTERNAL_SERVER,EXTERNAL_PORT)



    
def demo_data(message:str):
    print("\n{}\n".format(message))

def enter_panoramic_mode()->np.ndarray:
    """
    Creates a client and connects to the main camera client internally\n
    Sends signal to main camera to close the feed so this script can take control\n
    Enters panaramic mode and takes pictures and saves them at intervals\n
    Utilizes openCV to stich together saved images and saves them\n
    Sends signal to main camera to start the feed again to resume feed\n
    Kills the internal_client\n
    Returns the stiched image
    """

    #connecting to the server running in the main camera client script listening for messages
    internal_client = socket.socket(family=socket.AF_INET,type=socket.SOCK_STREAM)
    internal_client.connect(INTERNAL_ADDR)
    print("\n[CONNECTED]-> server: {} | port: {}".format(INTERNAL_SERVER,INTERNAL_ADDR))

    #messages to tell the main camera to STOP when this script starts, and START when this script ends
    start_message = "STOP".encode('utf-8')
    end_message = "START".encode('utf-8')

    #compute sizes and send start_message to the server
    start_msg_len = str(len(start_message)).encode('utf-8')
    start_msg_len += b' ' * (HEADER - len(start_msg_len))
    internal_client.send(start_msg_len)
    internal_client.send(start_message)


    print("\n[WAITING FIVE SECONDS FOR CAMERA TO RELEASE]")
    time.sleep(5)
    print("\n[STARTING CAMERA]\n")

    #reset servos to maintain a good image
    #reset_servos()

    i = 1 #turn the servo eight times, 22.5 degrees each, which translates to 45 degrees for the camera
    num_rotations = 8
    angle_change = 22.5

    #start camera
    camera = cv2.VideoCapture(0)
    image_list = list()
    while i <= num_rotations:
        ret, frame = camera.read()
        image_list.append(frame)
        i+=1
    
    camera.release()
    print("\n [RELEASED CAMERA]")

    #compute size and send START to the main camera client to let it know to restart
    end_msg_length = str(len(end_message)).encode('utf-8')
    end_msg_length += b' ' * (HEADER - len(end_msg_length))
    internal_client.send(end_msg_length)
    internal_client.send(end_message)
    print("\n[DISCONNECTING FROM INTERNAL SERVER...]")
    internal_client.shutdown(socket.SHUT_WR) #shutdown writing side
    print("\n[SUCCESSFULLY DISCONNECTED]")
    return image_list

def handle_client(connection:socket.socket,address):
    connected = True
    while connected:
        messageLength = connection.recv(HEADER).decode("utf-8")
        if messageLength:
            message = connection.recv(int(messageLength)).decode("utf-8")
            if message == "END":
                connected = False
            elif message == "PANO":
                stiched_image = enter_panoramic_mode()
            else:
                print("\nSending {} to send_servo_signal()".format(message))
                demo_data(message)
                #send_servo_signal(message)


    print("[ENDING CONNECTION]: {}".format(address))
    connection.close()
    return



def start_server():
    """
    Paramaters: None\n
    Function: Serves as the starting point for the function\n\t
    Creates the server that listens for the servo\n
    Creates a new thread for each connection
    """
    server = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    server.bind(ADDR)
    server.listen()
    print("[LISTENTING] on port {}\n".format(PORT))

    while True:
        connection, address = server.accept()
        print("[UPDATE] New Connection: {}".format(address))
        thread = threading.Thread(target=handle_client,args=[connection,address])
        thread.start()

start_server()

