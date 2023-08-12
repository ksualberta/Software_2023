#RUN ON MACBOOK

import gi
import signal
gi.require_version('Gst', '1.0')
gi.require_version("GstApp", "1.0")
from gi.repository import Gst, GLib
import cv2
import numpy as np
import threading
import queue

Gst.init(None)

logitech_brio = None
arducam1 = None

frame_queue = queue.Queue(maxsize=10)

#dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
#parameters =  cv2.aruco.DetectorParameters()
#detector = cv2.aruco.ArucoDetector(dictionary, parameters)

Logitech_Brio_Port = "7011"
Aduacam_Port = "7016" 

def shutdown(signum, frame):
    global logitech_brio, loop, arducam1
    print("\nShutting down...")
    
    # Send EOS to the pipeline
    # Stop the pipelines
    logitech_brio.set_state(Gst.State.NULL)
    arducam1.set_state(Gst.State.NULL)
    
    #cv2.destroyAllWindows()
    # Stop the main loop
    loop.quit()

def on_message(bus, message, pipeline):
    if message.type == Gst.MessageType.EOS:
        print("EOS received")
        shutdown(None, None)
    elif message.type == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        print(f"Error: {err}, {debug}")
        shutdown(None, None)

def gst_to_opencv(sample):
    """
    Converts Gstreamer sample to OpenCV image
    """
    print("Conversion Running")
    buf = sample.get_buffer()
    caps = sample.get_caps()
    arr = np.ndarray(
        (caps.get_structure(0).get_value("height"),
         caps.get_structure(0).get_value("width"),
         3),
        buffer=buf.extract_dup(0, buf.get_size()),
        dtype=np.uint8
    )
    return arr


def new_sample(appsink):
    """
    Callback to retrieve video frames from the appsink
    """
    print("new sample Running")
    sample = appsink.emit("pull-sample")
    frame = gst_to_opencv(sample)

    # Push the frame to the queue for processing
    try:
        frame_queue.put_nowait(frame)
    except queue.Full:
        # Drop the frame if the queue is full
        pass

    return Gst.FlowReturn.OK

def process_frames():
    """
    Worker thread function to process frames from the queue
    """
    while True:
        frame = frame_queue.get()
        cv2.imshow('frame', frame)

        
               # ArUco marker detection
        corners, ids, rejectedCandidates = detector.detectMarkers(frame)
        # If you want to draw the detected markers on the frame:
        frame_with_markers = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        
        # Display the frame with markers
        cv2.imshow('frame', frame_with_markers)
        cv2.waitKey(1)

def main():
    global logitech_brio, loop, arducam1

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    logitech_brio = Gst.parse_launch(
        "srtsrc uri=srt://192.168.1.3:" + Logitech_Brio_Port + "?mode=listener&latency=200 ! jpegparse ! jpegdec\
             ! autovideosink"
    )
    
    arducam1 = Gst.parse_launch(
        "srtsrc uri=srt://192.168.1.3:" + Aduacam_Port + "?mode=listener&latency=200 ! decodebin ! autovideosink"
    )

    logitech_brio.set_state(Gst.State.PLAYING)
    arducam1.set_state(Gst.State.PLAYING)

    logitech_brio_bus = logitech_brio.get_bus()
    logitech_brio_bus.add_signal_watch()
    logitech_brio_bus.connect("message", on_message, logitech_brio)

    arducam1_bus = arducam1.get_bus()
    arducam1_bus.add_signal_watch()
    arducam1_bus.connect("message", on_message, arducam1)

    #print("Reached Appsink")


    #appsink = logitech_brio.get_by_name("sink")
    #handler_id = appsink.connect("new-sample", new_sample)
    #worker_thread = threading.Thread(target=process_frames)
    #worker_thread.daemon = True  # This allows the thread to exit when the main program exits
    #worker_thread.start()


    loop = GLib.MainLoop()
    try:
        loop.run()
    except:
        pass

    logitech_brio.set_state(Gst.State.NULL)
    arducam1.set_state(Gst.State.NULL)
    #cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
