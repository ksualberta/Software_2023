import gi
import sys
import time
from gi.repository import GLib


gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject

ports = [7000]  # Add the ports you are using

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

def create_pipeline(port):
    # Create GStreamer pipeline
    pipeline = Gst.Pipeline()

    # Create elements
    srtserversrc = Gst.ElementFactory.make("srtserversrc", None)
    rtpdepay = Gst.ElementFactory.make("rtph264depay", None)
    dec = Gst.ElementFactory.make("avdec_h264", None)
    conv = Gst.ElementFactory.make("videoconvert", None)
    sink = Gst.ElementFactory.make("autovideosink", None)

    # Set element properties
    srtserversrc.set_property("uri", f"srt:localhost//:{7201}")
    srtserversrc.set_property("mode", 1)  # Listener mode

    # Add elements to pipeline and link them
    pipeline.add(srtserversrc)
    pipeline.add(rtpdepay)
    pipeline.add(dec)
    pipeline.add(conv)
    pipeline.add(sink)

    srtserversrc.link(rtpdepay)
    rtpdepay.link(dec)
    dec.link(conv)
    conv.link(sink)

    return pipeline

def main(argv):
    
    Gst.init(None)

    loop = GLib.MainLoop()

    pipelines = []
    for i, port in enumerate(ports):
        pipeline = create_pipeline(port)  # Use a different port for each camera
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
