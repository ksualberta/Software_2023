from tkinter import *
import cv2
from PIL import ImageTk,Image
import os

JPEGQUALITY = 25
ENCODEPARAM = [int(cv2.IMWRITE_JPEG_QUALITY), JPEGQUALITY]

def start():
    camera = cv2.VideoCapture(0)
    mainWindow = Tk()
    mainWindow.geometry(newGeometry="800x800")
    mainWindow.title("SPEAR MAIN FEED")
    mainFrame = Frame(master=mainWindow,)
    mainFrame.grid()
    mainFeedLabel = Label(master=mainFrame,width=400,height=400,borderwidth=3)
    mainFeedLabel.config(borderwidth=3)
    mainFeedLabel.grid(column=0)
    arucoDetectedLabel = Label(master=mainFrame,width=400,height=400,borderwidth=3,text="none found")
    arucoDetectedLabel.grid(column=1)

    while True:
        ret, frame = camera.read()
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image = ImageTk.PhotoImage(image=Image.fromarray(rgb_image))
        mainFeedLabel.config(image=image)

        mainWindow.update()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
       
    mainWindow.mainloop()
    camera.release()
    cv2.destroyAllWindows()

start()