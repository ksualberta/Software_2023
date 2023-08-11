import socket
import threading
import subprocess
from piservo import Servo
import numpy as np
import cv2
import pickle
import time

PORT = 5050
HEADER = 64 #the size of the buffer that declares the size of the buffer for the incoming message
SERVER = "0.0.0.0"
INTERNAL_SERVER = "192.168.1.2"
INTERNAL_PORT = 9050
EXTERNAL_SERVER = "192.168.1.1"
EXTERNAL_PORT = 9050
ADDR = (SERVER,PORT)
INTERNAL_ADDR = (INTERNAL_SERVER,INTERNAL_PORT)
EXTERNAL_ADDR = (EXTERNAL_SERVER,EXTERNAL_PORT)

#Use arrow keys for contorls 


servo_x = Servo(12) #Servo Pin Numbers
servo_y = Servo(13) #Servo Pin Numbers
servo_x_angle = 0
servo_y_angle = 0

def send_stiched_image(stiched_image:np.ndarray)->None:
    """
    Creates a client to deliver the stiched image to the base station\n
    Disconnects the client after sending the stiched image
    """
    print("\n[CONNECTED TO BASE STATION TO SEND IMAGE]")

    external_client = socket.socket(family=socket.AF_INET,type=socket.SOCK_STREAM)
    msg = pickle.dumps(stiched_image)
    msg_length = str(len(msg)).encode('utf-8')
    msg_length += b' ' * (HEADER - len(msg_length))

    external_client.send(msg_length)
    external_client.send(msg)
    print("[SENT IMAGE]")


def stitch_images(images:list)->np.ndarray:
    """
    Inputs the list of images to stich together\n
    Preproccesses the image and stiches them together\n
    Returns the stiched image
    """
    sticher = cv2.Stitcher.create()

    #resize images to be consistent
    for image in images:
        image = cv2.resize(image,(1280,720))

    (stitch_status,stiched_image) = sticher.stitch(images=images)

    if stitch_status != cv2.STITCHER_OK:
        return None
    else:
        return stiched_image
    
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
    reset_servos()

    i = 1 #turn the servo eight times, 22.5 degrees each, which translates to 45 degrees for the camera
    num_rotations = 8
    angle_change = 22.5

    #start camera
    camera = cv2.VideoCapture(0)
    image_list = list()
    while i <= num_rotations:
        ret, frame = camera.read()
        image_list.append(frame)
        send_servo_signal("+{}/x".format(angle_change))
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

    return stitch_images(images=image_list)

def reset_servos():
    """
    Used to reset the servo angles to 0 for each
    """
    servo_x.write(0)
    servo_y.write(0)

def send_servo_signal(message:str):
    """
    Sends angle measures to the servo\n
    If a command is send that exceeds limits, resets limits\n
    """
    global servo_x_angle
    global servo_y_angle

    messageList = message.split("/") #splits on the "/" to give the angle change in [0] and in axis in [1]
    angle_change = float(messageList[0])
    print("[X-ANGLE]: {}".format(servo_x_angle))
    print("[Y-ANGLE]: {}".format(servo_y_angle))
    print(["[CHANGE-ANGLE]: {}".format(angle_change)])
    match messageList[1]:
        case "x":
            servo_x_angle += angle_change
            try:
                servo_x.write(servo_x_angle)
            except:
                if servo_x_angle < 0:
                    servo_x.write(0)
                    servo_x_angle = 0
                else:
                    servo_x.write(180)
                    servo_x_angle = 180
        case "y":
            servo_y_angle += angle_change
            try:
                servo_y.write(servo_y_angle)
            except:
                if servo_y_angle < 0:
                    servo_y.write(0)
                    servo_y_angle = 0
                else:
                    servo_y.write(180)
                    servo_y_angle = 180
            
    print("Message recieved")
    return

def handle_client(connection:socket.socket,address):
    connected = True
    while connected:
        messageLength = connection.recv(HEADER).decode("utf-8")
        if messageLength:
            message = connection.recv(int(messageLength)).decode("utf-8")
            if message == "END":
                connected = False
            else:
                print("\nSending {} to send_servo_signal()".format(message))
                send_servo_signal(message)


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
    subprocess.call(['sudo','pigpiod']) #pigpiod is a utility which launches the pigpio library as a daemon.
    reset_servos()
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
