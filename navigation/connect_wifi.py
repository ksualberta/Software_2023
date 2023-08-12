import subprocess
import requests
import re
import socket

PORT = 9000
SERVER = "192.168.1.3"
ADDR = (SERVER,PORT)
HEADER = 64


def try_connection():
    """
    Continously runs sudo command to connect to desired ssid
    """
    ssid = "Isaac_Asimov"
    command = ['nmcli','device','wifi','connect',ssid,]

    connected = False

    while not connected:
        try:
            subprocess.run(args=command,check=True)
            connected = True

        except:
            print("[Trying to Connect...]\n")
            connected = False
    
    print("\n[Connected to ssid: {}]\n".format(ssid))

def return_response_body():
    print("[Sending GET request to webpage]\n")
    url = "http://10.10.11.1:80"
    response = requests.get(url)
    print("[RECIEVED REQUEST]")
    return response

def return_coordinates_altitude(response_body:str):
    print("[Parsing response..]")
    coordinates_searchParam = r'[-]*[0-9]*[.]+[0-9]*[a-zA-Z]{1}'
    altitude_searchParam = r'Altitude:\s[0-9]*[.][0-9]*'
    coordinates = re.findall(pattern=coordinates_searchParam,string=response_body)
    
    coordinates_without_direction = list()

    for coordinate in coordinates:
        coordinates_without_direction.append(float(re.findall(pattern=r'[-]*[0-9]*[.]+[0-9]*',string=coordinate)[0]))

    altitude = float(re.findall(pattern=altitude_searchParam,string=response_body)[0].split(" ")[1])
    return (coordinates_without_direction,altitude)

def start():
    client = socket.socket(family=socket.AF_INET,type=socket.SOCK_STREAM)
    client.connect(ADDR)
    print("[CONNECTED TO SERVER AT {}]\n".format(PORT))
    while True:
        try_connection()
        response = return_response_body()
        coordinates,altitude = return_coordinates_altitude(response.text)
        message = str(coordinates).encode('utf-8')
        message_length = str(len(message)).encode('utf-8')
        message_length += b' ' * (HEADER - len(message_length))

        client.send(message_length)
        client.send(message)

start()