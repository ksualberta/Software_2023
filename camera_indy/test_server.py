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
        "videotestsrc pattern=snow animation-mode=running-time is-live=true ! x264enc! mpegtsmux !"
        "srtserversink uri=srt://:5000/"
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