import gi
import sys
import time
from gi.repository import GLib


gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

devices = ["/dev/video2"]  # Add your device paths here

def bus_call(bus, message, loop, pipeline):
    t = message.type
    if t == Gst.MessageType.EOS:
        sys.stdout.write("End-of-stream\n")
        loop.quit()
    elif t == Gst.MessageType.ERROR:
        err, debug = message.parse_error()
        sys.stderr.write("Error: %s: %s\n" % (err, debug))
        pipeline.set_state(Gst.State.NULL)
        time.sleep(1)  # Wait a bit before trying to recover
        pipeline.set_state(Gst.State.PLAYING)  # Try to recover
    return True

def create_pipeline(device, port):
    # Create GStreamer pipeline
    pipeline = Gst.Pipeline()

    # Create elements
    src = Gst.ElementFactory.make("v4l2src", None)
    src.set_property("device", device)
    enc = Gst.ElementFactory.make("x264enc", None)
    rtppay = Gst.ElementFactory.make("rtph264pay", None)
    srtclientsink = Gst.ElementFactory.make("srtclientsink", None)
    local_sink = Gst.ElementFactory.make("autovideosink", None)

    # Set element properties
    srtclientsink.set_property("mode", 0)
    srtclientsink.set_property("uri", f"srt://192.168.1.1:{7000}")

    srtclientsink.set_property("latency", 200)

    # Add elements to pipeline and link them
    pipeline.add(src)
    pipeline.add(enc)
    pipeline.add(rtppay)
    pipeline.add(srtclientsink)
    

    src.link(enc)
    enc.link(rtppay)
    rtppay.link(srtclientsink)
    

    return pipeline

def main(argv):
 
    Gst.init(None)

    loop = GLib.MainLoop()

    pipelines = []
    for i, device in enumerate(devices):
        pipeline = create_pipeline(device, 7000 + i)  # Use a different port for each camera
        pipelines.append(pipeline)

        # Set up message handling
        bus = pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", bus_call, loop, pipeline)  # Pass pipeline to bus_call

        # Start pipeline
        pipeline.set_state(Gst.State.PLAYING)

    try:
        loop.run()
    except:
        pass

    # Clean up
    for pipeline in pipelines:
        pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    sys.exit(main(sys.argv))
