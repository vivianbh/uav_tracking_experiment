import gi
import numpy as np
import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

# Initialize GStreamer
Gst.init(None)
img_pub = rospy.Publisher("/exp/camera/raw", Image, queue_size=10)
img = Image()

def image_publisher(imgW, imgH, frame):
    header = Header(stamp = rospy.Time.now())
    header.frame_id = "object"
    header = header
    img.height = imgH
    img.width = imgW
    img.encoding = "bgr8"
    img.step = imgW*3
    img.data = np.array(frame).tobytes()
    img_pub.publish(img)

def on_new_sample(sink, loop):
    sample = sink.emit('pull-sample')
    buf = sample.get_buffer()
    caps = sample.get_caps()

    # Get frame width, height, and format
    structure = caps.get_structure(0)
    width = structure.get_value('width')
    height = structure.get_value('height')
    format = structure.get_value('format')

    # Check if the format is RGB
    if format != 'RGB':
        print(f"Unsupported format: {format}")
        return Gst.FlowReturn.OK

    # Extract the buffer
    success, map_info = buf.map(Gst.MapFlags.READ)
    if not success:
        return Gst.FlowReturn.ERROR

    try:
        # Create the NumPy array from the buffer data
        array = np.ndarray(
            (height, width, 3),
            buffer=map_info.data,
            dtype=np.uint8
        )

        # Convert from RGB to BGR for OpenCV
        frame = cv2.cvtColor(array, cv2.COLOR_RGB2BGR)
        image_publisher(width, height, frame)
        if rospy.is_shutdown():
            loop.quit()
    finally:
        buf.unmap(map_info)

    return Gst.FlowReturn.OK

def main():
    rospy.init_node("camera_stream", anonymous=False)
    
    pipeline = Gst.parse_launch(
        'rtspsrc location=rtsp://192.168.144.25:8554/main.264 latency=0 ! '
        'rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! '
        'video/x-raw, format=RGB ! appsink name=sink'
    )

    sink = pipeline.get_by_name('sink')
    sink.set_property('emit-signals', True)
    # Create a GLib Main Loop and set it to run
    loop = GLib.MainLoop()
    sink.connect('new-sample', on_new_sample, loop)

    pipeline.set_state(Gst.State.PLAYING)
    
    try:
        print('publish raw image from ip camera ...')
        loop.run()
    except:
        pass

    pipeline.set_state(Gst.State.NULL)

if __name__ == '__main__':
    main()

