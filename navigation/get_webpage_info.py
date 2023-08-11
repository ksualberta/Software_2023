import requests
from tkinter import *
import re
#import plotly.express as px
import matplotlib.pyplot as plt
import time
from PIL import ImageTk,Image

def getLocation(response_body:list)->list:
    coordinates_searchParam = r'[-]*[0-9]*[.]+[0-9]*[a-zA-Z]{1}'
    altitude_searchParam = r'Altitude:\s[0-9]*[.][0-9]*'
    coordinates = re.findall(pattern=coordinates_searchParam,string=response_body)
    
    coordinates_without_direction = list()

    for coordinate in coordinates:
        coordinates_without_direction.append(float(re.findall(pattern=r'[-]*[0-9]*[.]+[0-9]*',string=coordinate)[0]))

    altitude = float(re.findall(pattern=altitude_searchParam,string=response_body)[0].split(" ")[1])
    return (coordinates_without_direction,altitude)

def demoLocation():
    data = {
    'City': ['Calgary', 'Edmonton', 'Red Deer', 'Lethbridge', 'Fort McMurray'],
    'Latitude': [51.0486, 53.5444, 52.2681, 49.6935, 56.7268],
    'Longitude': [-114.0708, -113.4909, -113.8113, -112.8418, -111.3809]
    }
    response = requests.get("http://127.0.0.1:5500/Software_2023/website_interface_example/index.html")
    rootWindow = Tk()
    rootWindow.title("Sample Screen")
    rootWindow.geometry("680x480")
    photo = Widget(master=rootWindow)
    photo.grid(column=0,row=0)
    

    response_body = response.content.decode("utf-8")
    coordinates = getLocation(response_body)[0]
    plt.figure(figsize=(10,6))
    plt.scatter(x=[coordinates[0]],y=[coordinates[1]])
    plt.annotate(text="Goal\nLat:{}\nLon:{}".format(coordinates[0],coordinates[1]),xy=[coordinates[0],coordinates[1]])
    fig = plt.gcf()
    fig.canvas.draw()
    #np.array(fig.canvas.)
    image = Image.fromarray()
    photoImage = ImageTk.PhotoImage(image)
    rootWindow.mainloop()

demoLocation()
