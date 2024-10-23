#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import gi
import threading
import socket

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

Gst.init(None)


class CameraImagePublisher(Node):
    def __init__(self):
        super().__init__('camera_image_publisher')

        # Subscribe to ROS2 image topic
        self.image_subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.camera_image_callback,
            10)
        self.bridge = CvBridge()

        # Set up GStreamer pipeline variables
        self.number_frames = 0
        self.fps = 15
        self.duration = 1 / self.fps * Gst.SECOND
        self.pipe = "appsrc name=source is-live=true block=true format=GST_FORMAT_TIME " \
                    "caps=video/x-raw,format=BGR,width=640,height=480,framerate={}/1 " \
                    "! videoconvert ! video/x-raw,format=I420 " \
                    "! jpegenc " \
                    "! rtpjpegpay " \
                    "! udpsink host=127.0.0.1 port=1234".format(self.fps)
        self.pipeline = None
        self.appsrc = None
        self.loop = None
        self.buffer_lock = threading.Lock()
        self.current_frame = None
        self.pipeline_started = False

    def start_pipeline(self):
        try:
            self.pipeline = Gst.parse_launch(self.pipe)
            self.pipeline.set_state(Gst.State.READY)
            self.pipeline.set_state(Gst.State.PLAYING)
            self.appsrc = self.pipeline.get_by_name('source')
            self.appsrc.connect('need-data', self.on_need_data)
            self.loop = GLib.MainLoop()

            # Start GLib loop in a separate thread to avoid blocking ROS2
            threading.Thread(target=self.run_loop, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f'Failed to start GStreamer pipeline: {e}')

    def run_loop(self):
        self.loop.run()

    def on_need_data(self, src, length):
        with self.buffer_lock:
            if self.current_frame is not None:
                self.get_logger().info(f'Pushing frame {self.number_frames}')
                frame = self.current_frame
                frame = cv2.resize(frame, (640, 480))

                # Convert frame to bytes
                data = frame.tobytes()

                # Create and push buffer to GStreamer pipeline
                buf = Gst.Buffer.new_allocate(None, len(data), None)
                buf.fill(0, data)
                buf.duration = self.duration
                timestamp = self.number_frames * self.duration
                buf.pts = buf.dts = int(timestamp)
                buf.offset = timestamp
                self.number_frames += 1
                retval = src.emit('push-buffer', buf)

                if retval != Gst.FlowReturn.OK:
                    self.get_logger().error(f'Error pushing buffer: {retval}')
            else:
                self.get_logger().warning('No current frame to push to GStreamer')

    def camera_image_callback(self, msg):
        # Convert ROS2 Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        # Store the current frame for GStreamer pipeline
        with self.buffer_lock:
            self.current_frame = cv_image

        self.get_logger().info('Received image from ROS2 topic')

        # Start GStreamer pipeline after receiving the first frame
        if not self.pipeline_started:
            self.get_logger().info('Starting GStreamer pipeline')
            self.start_pipeline()
            self.pipeline_started = True

    def stop_pipeline(self):
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
            self.pipeline = None
            self.loop.quit()


def main(args=None):
    rclpy.init(args=args)

    # Create and spin the ROS2 node
    camera_image_publisher = CameraImagePublisher()
    rclpy.spin(camera_image_publisher)

    # Shutdown ROS2 node
    camera_image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
