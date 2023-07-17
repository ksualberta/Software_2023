from tkinter import *
import cv2
from PIL import ImageTk,Image
import os

JPEGQUALITY = 25
ENCODEPARAM = [int(cv2.IMWRITE_JPEG_QUALITY), JPEGQUALITY]

def start():
    camera = cv2.VideoCapture(0)
    mainWindow = Tk(screenName="Sample Window")
    mainWindow.maxsize(width=1000,height=1000)
    mainFeedLabel = Label(master=mainWindow)
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