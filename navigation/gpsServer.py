import socket
import threading

PORT = 7021
SERVER = "0.0.0.0"
ADDR = (SERVER,PORT)
HEADER = 64

server = socket.socket(family=socket.AF_INET,type=socket.SOCK_STREAM)
server.bind(ADDR)
server.listen()


def handle_client(connection:socket.socket):
    connected = True
    while connected:
        message_length = int(connection.recv(HEADER).decode('utf-8'))

        if message_length:
            message = b''
            while len(message) < message_length:
                message += connection.recv(message_length - len(message))
        
            message = message.decode('utf-8')
            print(message)


while True:
    conn,addr = server.accept()
    new_thread = threading.Thread(target=handle_client,args=[conn])
    new_thread.start()
