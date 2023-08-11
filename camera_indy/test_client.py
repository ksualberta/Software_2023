import gi
import signal
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)

pipeline = None

Logitech_Brio_Port = "7060"
Aduacam_Port = "7050" 

def shutdown(signum, frame):
    global logitech_brio, loop, arducam1
    print("\nShutting down...")
    
    # Send EOS to the pipeline
    # Stop the pipelines
    logitech_brio.set_state(Gst.State.NULL)
    arducam1.set_state(Gst.State.NULL)
    
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

def main():
    global logitech_brio, loop, arducam1

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

#    pipeline = Gst.parse_launch(
#        "srtsrc uri=srt://192.168.1.3:7030?mode=listener&latency=200 ! decodebin ! autovideosink"
#    )

    logitech_brio = Gst.parse_launch(
        "srtsrc uri=srt://192.168.1.3:" + Logitech_Brio_Port + "?mode=listener&latency=200 ! jpegdec ! autovideosink"
    )
    
    arducam1 = Gst.parse_launch(
        "srtsrc uri=srt://192.168.1.3:" + Aduacam_Port + "?mode=listener&latency=200 ! jpegdec ! autovideosink"
    )

    logitech_brio.set_state(Gst.State.PLAYING)
    arducam1.set_state(Gst.State.PLAYING)

    logitech_brio_bus = logitech_brio.get_bus()
    logitech_brio_bus.add_signal_watch()
    logitech_brio_bus.connect("message", on_message, logitech_brio)

    arducam1_bus = arducam1.get_bus()
    arducam1_bus.add_signal_watch()
    arducam1_bus.connect("message", on_message, arducam1)

    loop = GLib.MainLoop()
    try:
        loop.run()
    except:
        pass

    logitech_brio.set_state(Gst.State.NULL)
    arducam1.set_state(Gst.State.NULL)

if __name__ == "__main__":
    main()
