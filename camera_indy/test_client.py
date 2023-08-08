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
    
    # Give it some time to handle the EOS
    GLib.timeout_add(300, lambda: pipeline.set_state(Gst.State.NULL))
    GLib.timeout_add(500, lambda: loop.quit())

def main():
    global pipeline, loop

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    pipeline = Gst.parse_launch(
        "srtclientsrc uri=srt://127.0.0.1:5000/ ! queue ! tsparse ! tsdemux ! decodebin ! glimagesink"
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
