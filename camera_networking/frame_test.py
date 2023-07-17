from tkinter import *
import cv2
from PIL import ImageTk,Image
import os

JPEGQUALITY = 25
ENCODEPARAM = [int(cv2.IMWRITE_JPEG_QUALITY), JPEGQUALITY]

def start():
    camera = cv2.VideoCapture(0)
    mainWindow = Tk()
    mainWindow.geometry(newGeometry="1000x1000")
    mainWindow.title("SPEAR MAIN FEED")
    mainFrame = Frame(master=mainWindow,borderwidth=3)
    mainFrame.grid(column=0)
    mainFeedLabel = Label(master=mainFrame)
    mainFeedLabel.config(borderwidth=2)
    mainFeedLabel.grid()
    while True:
        ret, frame = camera.read()
        image = cv2.imencode(".jpg",frame,ENCODEPARAM)
        mainFeedPath = os.path.join(os.getcwd(),"mainFeed.jpg")
        cv2.imwrite(mainFeedPath,frame,params=ENCODEPARAM)
        cv2.imshow("Message",frame)
        mainFeedImage = Image.open(mainFeedPath).resize((250,250),Image.LANCZOS)
        mainFeedLabel.config(image=ImageTk.PhotoImage(mainFeedImage))

        mainWindow.update()
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        
       
    mainWindow.mainloop()
    camera.release()
    cv2.destroyAllWindows()

start()