import subprocess
import requests
import re
import threading

PORT = 9000
SERVER = ""



def try_connection():
    """
    Continously runs sudo command to connect to desired ssid
    """
    ssid = "AP"
    command = ['nmcli','device','wifi','connect',ssid,'password','Abhinav1']

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
    url = "http://192.168.1.1:5500/Software_2023/website_interface_example/"
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
    try_connection()
    response = return_response_body()
    print(response.text)
    print(type(response.text))
    coordinates,altitude = return_coordinates_altitude(response.text)
    print(coordinates)

start()