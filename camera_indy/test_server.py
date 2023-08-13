import gi
import signal
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

HD_CAMERA_1 = None
HD_CAMERA_2 = None
logitech_brio = None
Logitech_Brio_ID = "2"
Logitech_Brio_Port = "7051"
HD_CAMERA_1_ID = "0"
HD_CAMERA_1_Port = "7052"
HD_CAMERA_2_ID = "6"
HD_CAMERA_2_Port = "7053"


def shutdown(signum, frame):
    global HD_CAMERA_1, loop, logitech_brio, HD_CAMERA_2
    print("\nShutting down...")
    
    # Send EOS to the pipeline
    HD_CAMERA_1.send_event(Gst.Event.new_eos())
    HD_CAMERA_2.send_event(Gst.Event.new_eos())
    logitech_brio.send_event(Gst.Event.new_eos())
    
    # Wait for the EOS to be processed
    logi_bus = logitech_brio.get_bus()
    HD_CAMERA_1_bus = HD_CAMERA_1.get_bus()
    HD_CAMERA_2_bus = HD_CAMERA_2.get_bus()
    logi_msg = logi_bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS)
    HD_CAMERA_1_msg = HD_CAMERA_1_bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS)
    HD_CAMERA_2_msg = HD_CAMERA_2_bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS)
    if logi_msg:
        print("EOS received Logitech")
    
    if HD_CAMERA_1_msg:
        print("EOS received HD_CAMERA 1")

    if HD_CAMERA_2_msg:
        print("EOS received HD_CAMERA 2")


    
    # Set pipeline to NULL state
    HD_CAMERA_1.set_state(Gst.State.NULL)
    HD_CAMERA_2.set_state(Gsr.State.NULL)
    logitech_brio.set_state(Gst.State.NULL)
    
    # Stop the main loop
    loop.quit()

def main():
    global HD_CAMERA_1, loop, logitech_brio, HD_CAMERA_2

    # Signal handling for graceful shutdown
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    HD_CAMERA_1 = Gst.parse_launch(
    "v4l2src device=/dev/video" + HD_CAMERA_1_ID +" ! image/jpeg,width=1280,height=720,framerate=25/1 ! \
        srtserversink uri=srt://192.168.1.3:"+ HD_CAMERA_1_Port +"?latency=20/"
    )

    HD_CAMERA_2 = Gst.parse_launch(
    "v4l2src device=/dev/video" + HD_CAMERA_2_ID +" ! image/jpeg,width=1280,height=720,framerate=25/1 ! \
        srtserversink uri=srt://192.168.1.3:"+ HD_CAMERA_2_Port +"?latency=20/"
    )

    logitech_brio = Gst.parse_launch(
    "v4l2src device=/dev/video" + Logitech_Brio_ID +" ! image/jpeg,width=1280,height=720,framerate=25/1 ! \
        srtserversink uri=srt://192.168.1.3:"+ Logitech_Brio_Port +"?latency=20/"
    )

    HD_CAMERA_1.set_state(Gst.State.PLAYING)
    HD_CAMERA_2.set_state(Gst.State.PLAYING)
    logitech_brio.set_state(Gst.State.PLAYING)

    loop = GLib.MainLoop()
    try:
        loop.run()
    except:
        pass

    HD_CAMERA_1.set_state(Gst.State.NULL)
    HD_CAMERA_2.set_state(Gst.State.NULL)
    logitech_brio.set_state(Gst.State.NULL)

if __name__ == "__main__":
    main()