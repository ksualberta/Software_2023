#Run


import socket
import keyboard
import random

PORT = 5050
SERVER = socket.gethostbyname(socket.gethostname())
ADDR = (SERVER,PORT)


def start(x_speed:int,y_speed:int):
    client = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    client.connect(ADDR)

    connected = True
    print("\nListening for input:")
    message="start"
    #using to detect duplicate messages
    while connected:
        input = keyboard.read_key()
        if input=="k":
            message = "END"
            connected = False
        elif input == "p":
            message = "PANO"
        elif input == "right":
            message = "-{}/x".format(x_speed)
        elif input == "left":
            message = "+{}/x".format(x_speed)
        elif input == "up":
            message = "+{}/y".format(y_speed)
        elif input == "down":
            message = "-{}/y".format(y_speed)
        else:
            message = ""

        if message != "":    
            print("Message:{} [SENT]\n".format(message))
            message = message.encode('utf-8')
            messageLength = str(len(message)).encode('utf-8')
            messageLength += b' ' * (64 - len(messageLength))
            client.send(messageLength)
            client.send(message)


start(x_speed=2.5,y_speed=2.5) #speed should be in angle measures


