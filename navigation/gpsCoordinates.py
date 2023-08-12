#GET DATA FROM PI RUN ON PI
#Run in Sudo

import serial
import socket

PORT = 7021
SERVER = "192.168.1.4" #Linux VM
ADDR = (SERVER,PORT)
HEADER = 64

command = serial.Serial('/dev/ttyUSB0',4800, timeout=5 )

client = socket.socket(family=socket.AF_INET,type=socket.SOCK_STREAM)
client.connect(ADDR)
#print("[CONNECTED TO SERVER AT {}]\n".format(PORT)) broken code

def get_standard_coordinate(coordinate:str,direction:str)->str:
    dd = float(coordinate[:2])
    mm = float(coordinate[2:])

    decimal_minutes = mm/60

    decimate_coordinate = dd + decimal_minutes

    if direction == "S" or direction == "W":
        decimate_coordinate *= -1

    return decimate_coordinate

while True:
    input = command.readLine()
    splitInputs = input.split(",")

    if splitInputs[0] == "$GPGGA":
        lat = splitInputs[2]
        lat_direction = splitInputs[3]
        longitude = splitInputs[4]
        long_direction = splitInputs[5]
        altitude = splitInputs[9]

        lat = get_standard_coordinate(lat,lat_direction)
        longitude = get_standard_coordinate(longitude,long_direction)
        message = "({},{},{})".format(lat,longitude,altitude)
        message = message.encode('utf-8')
        message_length = str(len(message)).encode('utf-8')
        message_length += b' ' * (HEADER - len(message_length))

        client.send(message_length)
        client.send(message)