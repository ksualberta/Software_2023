import socket
import threading
import subprocess
from piservo import Servo

PORT = 5050
HEADER = 64 #the size of the buffer that declares the size of the buffer for the incoming message
SERVER = "0.0.0.0"
INTERNAL_SERVER = "192.168.1.2"
INTERNAL_PORT = 9050
ADDR = (SERVER,PORT)
INTERNAL_ADDR = (INTERNAL_SERVER,INTERNAL_PORT)

servo_x = Servo(12)
servo_y = Servo(13)
servo_x_angle = 0
servo_y_angle = 0

def enter_panoramic_mode():
    """
    Creates a client and connects to the main camera client internally\n
    Sends signal to main camera to close the feed so this scrip can take control\n
    Enters panaramic mode and takes pictures and saves them at intervals\n
    Utilizes openCV to stich together saved images and saves them\n
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
            elif message == "PANO":
                enter_panoramic_mode()
            else:
                print("\nSending {} to send_servo_signal()".format(message))
                send_servo_signal(message)


    print("[ENDING CONNECTION]: {}".format(address))
    connection.close()
    return



def start_server():
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
