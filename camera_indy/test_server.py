import gi
import signal
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

pipeline = None

def shutdown(signum, frame):
    global pipeline, loop
    print("\nShutting down...")
    
    # Send EOS to the pipeline
    pipeline.send_event(Gst.Event.new_eos())
    
    # Wait for the EOS to be processed
    bus = pipeline.get_bus()
    msg = bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS)
    if msg:
        print("EOS received")
    
    # Set pipeline to NULL state
    pipeline.set_state(Gst.State.NULL)
    
    # Stop the main loop
    loop.quit()

def main():
    global pipeline, loop

    # Signal handling for graceful shutdown
    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    pipeline = Gst.parse_launch(
        "v4l2src device=/dev/video4 ! video/x-h264,width=1920.height=1080,framerate=30/1 !"
        " srtserversink uri=srt://:5000/"
    )

    pipeline.set_state(Gst.State.PLAYING)

    loop = GLib.MainLoop()
    try:
        loop.run()
    except:
        pass

    pipeline.set_state(Gst.State.NULL)

if __name__ == "__main__":
    main()