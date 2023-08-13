import socket
import threading
from piservo import Servo

PORT = 5050
SERVER = "0.0.0.0"
ADDR = (SERVER, PORT)

servo_x = Servo(12)  # Servo Pin Numbers
servo_y = Servo(13)  # Servo Pin Numbers
servo_x_angle = 0
servo_y_angle = 0


def reset_servos():
    servo_x.write(0)
    servo_y.write(0)


def send_servo_signal(message: str):
    global servo_x_angle
    global servo_y_angle

    messageList = message.split("/")
    angle_change = float(messageList[0])

    if messageList[1] == "x":
        servo_x_angle += angle_change
        if 0 <= servo_x_angle <= 180:
            servo_x.write(servo_x_angle)
    elif messageList[1] == "y":
        servo_y_angle += angle_change
        if 0 <= servo_y_angle <= 180:
            servo_y.write(servo_y_angle)


def handle_client(connection: socket.socket, address):
    connected = True
    while connected:
        messageLength = connection.recv(64).decode("utf-8")
        if messageLength:
            message = connection.recv(int(messageLength)).decode("utf-8")
            if message == "END":
                connected = False
            else:
                send_servo_signal(message)

    connection.close()


def start_server():
    reset_servos()
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(ADDR)
    server.listen()
    print("[LISTENING] on port {}".format(PORT))

    while True:
        connection, address = server.accept()
        thread = threading.Thread(
            target=handle_client, args=[connection, address])
        thread.start()


start_server()
