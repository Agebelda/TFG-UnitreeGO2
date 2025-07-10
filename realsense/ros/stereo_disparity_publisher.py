#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import pyrealsense2 as rs
import numpy as np
import time

class StereoDisparityPublisher(Node):
    def __init__(self):
        super().__init__('stereo_disparity_publisher')

        self.bridge = CvBridge()
        self.pub_left = self.create_publisher(Image, '/stereo_virtual/left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, '/stereo_virtual/right/image_raw', 10)

        # Parámetros de cámara
        self.baseline = 0.065           # 6.5 cm
        self.focal_length_px = 600.0
        self.image_width = 640
        self.image_height = 480
        self.fps = 15                   # Publicar a máx 15 Hz

        # Inicializar RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.image_width, self.image_height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.image_width, self.image_height, rs.format.bgr8, self.fps)
        self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)

        self.get_logger().info("Publicando imágenes estéreo...")
        self.process_loop()

    def process_loop(self):
        self.get_logger().info("Entrando al bucle principal...")
        while rclpy.ok():
            self.get_logger().debug("Iteración del bucle")

            try:
                frames = self.pipeline.wait_for_frames(timeout_ms=200)
            except RuntimeError:
                self.get_logger().warn("No llegaron frames a tiempo, saltando iteración")
                time.sleep(0.01)
                continue

            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()

            if not depth_frame or not color_frame:
                self.get_logger().warn("Frame de color o profundidad no válido")
                continue

            # Procesar imágenes
            depth_image = np.asanyarray(depth_frame.get_data()).astype(np.float32)
            color_image = np.asanyarray(color_frame.get_data())

            depth_m = depth_image / 1000.0
            disparity = (self.baseline * self.focal_length_px) / np.where(depth_m > 0, depth_m, np.inf)
            disparity = disparity.astype(np.int32)

            h, w = depth_image.shape
            X, Y = np.meshgrid(np.arange(w), np.arange(h))

            Xl = np.clip(X - disparity, 0, w - 1)
            Xr = np.clip(X + disparity, 0, w - 1)
            left_image = color_image[Y, Xl]
            right_image = color_image[Y, Xr]

            # Publicar imágenes
            now = self.get_clock().now().to_msg()
            msg_left = self.bridge.cv2_to_imgmsg(left_image, encoding="bgr8")
            msg_right = self.bridge.cv2_to_imgmsg(right_image, encoding="bgr8")
            msg_left.header.stamp = now
            msg_right.header.stamp = now
            msg_left.header.frame_id = "left_cam"
            msg_right.header.frame_id = "right_cam"

            self.pub_left.publish(msg_left)
            self.pub_right.publish(msg_right)

            self.get_logger().debug("Imágenes publicadas")
            time.sleep(1 / self.fps)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = StereoDisparityPublisher()
    except KeyboardInterrupt:
        print("Nodo interrumpido")
    finally:
        rclpy.shutdown()
