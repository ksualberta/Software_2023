import gi
import time
from threading import Thread
import subprocess

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

# The IP address of your receiving device (laptop/client)
laptop_ip = 'YOUR_LAPTOP_IP'

# The port numbers to use for each camera
ports = [5000, 5001, 5002, 5003]

# The /dev/videoX device numbers for each camera
device_numbers = [0, 1, 2, 3]

pipelines = []

def create_pipeline(device_number, port):
    pipeline_str = (f'v4l2src device=/dev/video{device_number} ! '
                    'videoconvert ! x264enc tune=zerolatency ! '
                    f'rtph264pay ! udpsink host={laptop_ip} port={port}')
    pipeline = Gst.parse_launch(pipeline_str)

    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect('message::error', on_error, pipeline)

    return pipeline

def on_error(bus, message, pipeline):
    print(f"Error on pipeline for device /dev/video{pipeline.device_number}. Restarting...")
    pipeline.set_state(Gst.State.NULL)
    time.sleep(1)
    pipeline.set_state(Gst.State.PLAYING)

for device_number, port in zip(device_numbers, ports):
    pipeline = create_pipeline(device_number, port)
    pipeline.device_number = device_number  # Store the device number for error handling
    pipelines.append(pipeline)

def network_monitor():
    while True:
        try:
            subprocess.check_call(['ping', '-c', '1', laptop_ip], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            time.sleep(10)
        except subprocess.CalledProcessError:
            print('Network down. Restarting pipelines...')
            for pipeline in pipelines:
                pipeline.set_state(Gst.State.NULL)
                time.sleep(1)
                pipeline.set_state(Gst.State.PLAYING)
            time.sleep(5)  # Wait for a while before checking again

monitor_thread = Thread(target=network_monitor)
monitor_thread.start()

for pipeline in pipelines:
    pipeline.set_state(Gst.State.PLAYING)

GLib.MainLoop().run()
