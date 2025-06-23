#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import pyrealsense2 as rs
import numpy as np

class StereoDisparityPublisher(Node):
    def __init__(self):
        super().__init__('stereo_disparity_publisher')

        self.bridge = CvBridge()
        self.pub_left = self.create_publisher(Image, '/stereo_virtual/left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, '/stereo_virtual/right/image_raw', 10)

        # Parámetros
        self.baseline = 0.065  # 6.5 cm
        self.focal_length_px = 600.0
        self.image_width = 640
        self.image_height = 480

        # Inicializar RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.image_width, self.image_height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.image_width, self.image_height, rs.format.bgr8, 30)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        self.get_logger().info("📡 Publicando imágenes estéreo desde disparidad...")

        try:
            self.process_loop()
        except KeyboardInterrupt:
            self.get_logger().info("🛑 Interrumpido por el usuario.")
        finally:
            self.pipeline.stop()

    def process_loop(self):
        while rclpy.ok():
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)

            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32)
            color_image = np.asanyarray(color_frame.get_data())

            # Calcular disparidad
            depth_m = depth_image / 1000.0
            disparity = (self.baseline * self.focal_length_px) / np.where(depth_m > 0, depth_m, np.inf)
            disparity = disparity.astype(np.int32)

            h, w = depth_image.shape
            x = np.arange(w)
            y = np.arange(h)
            X, Y = np.meshgrid(x, y)

            # Imagen izquierda
            Xl = X - disparity
            Xl = np.clip(Xl, 0, w - 1)
            left_image = color_image[Y, Xl]

            # Imagen derecha
            Xr = X + disparity
            Xr = np.clip(Xr, 0, w - 1)
            right_image = color_image[Y, Xr]

            # Publicar
            now = self.get_clock().now().to_msg()
            msg_left = self.bridge.cv2_to_imgmsg(left_image, encoding="bgr8")
            msg_right = self.bridge.cv2_to_imgmsg(right_image, encoding="bgr8")
            msg_left.header.stamp = now
            msg_right.header.stamp = now
            self.pub_left.publish(msg_left)
            self.pub_right.publish(msg_right)


def main(args=None):
    rclpy.init(args=args)
    node = StereoDisparityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
