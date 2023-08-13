import socket
import keyboard

PORT = 5050
SERVER = socket.gethostbyname(socket.gethostname())
ADDR = (SERVER, PORT)


def start(x_speed: int, y_speed: int):
    client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client.connect(ADDR)

    connected = True
    while connected:
        key_input = keyboard.read_key()
        if key_input == "k":
            message = "END"
            connected = False
        elif key_input == "right":
            message = "-{}/x".format(x_speed)
        elif key_input == "left":
            message = "+{}/x".format(x_speed)
        elif key_input == "up":
            message = "+{}/y".format(y_speed)
        elif key_input == "down":
            message = "-{}/y".format(y_speed)
        else:
            message = ""

        if message:
            message = message.encode('utf-8')
            messageLength = str(len(message)).encode('utf-8')
            messageLength += b' ' * (64 - len(messageLength))
            client.send(messageLength)
            client.send(message)


start(x_speed=2.5, y_speed=2.5)
