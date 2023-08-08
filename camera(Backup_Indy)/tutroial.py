import gi
import subprocess
import time
from threading import Thread

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)

# The IP address of your laptop
laptop_ip = 'YOUR_LAPTOP_IP'

# The port numbers to use for each camera
ports = [5000, 5001, 5002, 5003]  # Add or remove port numbers as needed

# The /dev/videoX device numbers for each camera
device_numbers = [0, 1, 2, 3]  # Add or remove device numbers as needed

# A list to hold the pipelines
pipelines = []

def create_pipeline(device_number, port):
    # Build the pipeline
    pipeline = Gst.parse_launch(f'v4l2src device=/dev/video{device_number} ! videoconvert ! x264enc tune=zerolatency ! rtph264pay ! udpsink host={laptop_ip} port={port}')

    # Create a bus and set up an error message handler
    bus = pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect('message::error', on_error, pipeline)

    return pipeline

def on_error(bus, message, pipeline):
    # Print the error details
    gerror, debug_info = message.parse_error()
    print(f'Error received from element {message.src.get_name()}: {gerror.message}')
    print(f'Debugging information: {debug_info if debug_info else "none"}')

    # Set the pipeline to NULL and remove it from the list
    pipeline.set_state(Gst.State.NULL)
    pipelines.remove(pipeline)

    # Wait a moment
    time.sleep(1)

    # Try to create a new pipeline and add it to the list
    device_number = int(pipeline.get_by_name('v4l2src0').get_property('device')[-1])
    port = int(pipeline.get_by_name('udpsink0').get_property('port'))
    new_pipeline = create_pipeline(device_number, port)
    pipelines.append(new_pipeline)
    new_pipeline.set_state(Gst.State.PLAYING)

# Create a pipeline for each camera
for device_number, port in zip(device_numbers, ports):
    pipeline = create_pipeline(device_number, port)
    pipelines.append(pipeline)

def network_monitor():
    while True:
        try:
            # Ping the laptop IP address
            subprocess.check_call(['ping', '-c', '1', laptop_ip], stdout=subprocess.DEVNULL)

            # If the ping was successful, wait a while before the next one
            time.sleep(10)

        except subprocess.CalledProcessError:
            print('Network is down. Waiting for it to come back up...')

            # If the ping failed, wait a bit and then try again
            while True:
                try:
                    time.sleep(1)
                    subprocess.check_call(['ping', '-c', '1', laptop_ip], stdout=subprocess.DEVNULL)
                    break
                except subprocess.CalledProcessError:
                    pass

            print('Network is back up. Restarting pipelines...')

            # When the network comes back up, restart all the pipelines
            for pipeline in pipelines:
                pipeline.set_state(Gst.State.NULL)
                pipeline.set_state(Gst.State.PLAYING)

# Start the network monitor in a separate thread
monitor_thread = Thread(target=network_monitor)
monitor_thread.start()

# Start playing all the pipelines
for pipeline in pipelines:
    pipeline.set_state(Gst.State.PLAYING)

# Start the GLib main loop
GLib.MainLoop().run()
