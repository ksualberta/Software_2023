import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

# The port numbers for the camera streams to receive
ports = [5000, 5001, 5002, 5003]  # Adjust based on the number of cameras

pipelines = []

def create_pipeline(port):
    pipeline_str = (f'udpsrc port={port} ! '
                    'application/x-rtp, encoding-name=H264 ! '
                    'rtph264depay ! h264parse ! avdec_h264 ! autovideosink')
    pipeline = Gst.parse_launch(pipeline_str)
    return pipeline

for port in ports:
    pipeline = create_pipeline(port)
    pipelines.append(pipeline)

for pipeline in pipelines:
    pipeline.set_state(Gst.State.PLAYING)

GLib.MainLoop().run()
