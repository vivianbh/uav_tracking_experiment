import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GObject
import rospy

# Initialize GStreamer
Gst.init(None)

def main():
    # Create the pipeline
    pipeline = Gst.parse_launch(
        'rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=0 ! '
        'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink'
    )

    # Start playing
    pipeline.set_state(Gst.State.PLAYING)

    # Wait until error or EOS (End of Stream)
    bus = pipeline.get_bus()
    msg = bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.ERROR | Gst.MessageType.EOS)

    # Free resources
    pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    main()

