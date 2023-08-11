import gi
import signal
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

arducam1 = None
logitech_brio = None
Logitech_Brio_ID = "4"
Logitech_Brio_Port = "7060"
Aduacam_ID = "2"
Aduacam_Port = "7050"  


def shutdown(signum, frame):
    global arducam1, loop, logitech_brio
    print("\nShutting down...")
    
    # Send EOS to the pipeline
    arducam1.send_event(Gst.Event.new_eos())
    logitech_brio.send_event(Gst.Event.new_eos())
    
    # Wait for the EOS to be processed
    logi_bus = logitech_brio.get_bus()
    arducam1_bus = arducam1.get_bus()
    logi_msg = logi_bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS)
    arducam1_msg = arducam1_bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS)
    if logi_msg:
        print("EOS received Logitech")
    
    if arducam1_msg:
        print("EOS received Arducam")

    
    # Set pipeline to NULL state
    arducam1.set_state(Gst.State.NULL)
    logitech_brio.set_state(Gst.State.NULL)
    
    # Stop the main loop
    loop.quit()

def main():
    global arducam1, loop, logitech_brio

    # Signal handling for graceful shutdown
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    arducam1 = Gst.parse_launch(
    "v4l2src device=/dev/video" + Aduacam_ID +" ! video/x-h264,width=1920,height=1080,framerate=30/1 ! \
        srtserversink uri=srt://192.168.1.3:"+ Aduacam_Port +"?latency=20/"
        )

    logitech_brio = Gst.parse_launch(
    "v4l2src device=/dev/video" + Logitech_Brio_ID +" ! image/jpeg,width=1920,height=1080,framerate=24/1 ! \
        srtserversink uri=srt://192.168.1.3:"+ Logitech_Brio_Port +"?latency=20/"
    )

    arducam1.set_state(Gst.State.PLAYING)
    logitech_brio.set_state(Gst.State.PLAYING)

    loop = GLib.MainLoop()
    try:
        loop.run()
    except:
        pass

    arducam1.set_state(Gst.State.NULL)
    logitech_brio.set_state(Gst.State.NULL)

if __name__ == "__main__":
    main()