#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import pyrealsense2 as rs
import numpy as np
import open3d as o3d
import open3d.visualization.rendering as rendering  # type: ignore
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class StereoVirtualPublisher(Node):
    def __init__(self):
        super().__init__('stereo_virtual_publisher')

        self.bridge = CvBridge()
        self.pub_left = self.create_publisher(Image, '/stereo_virtual/left/image_raw', 10)
        self.pub_right = self.create_publisher(Image, '/stereo_virtual/right/image_raw', 10)

        # Parámetros
        self.image_width = 640
        self.image_height = 480

        # Inicializar pipeline
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.image_width, self.image_height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.image_width, self.image_height, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(config)

        # Alinear profundidad al color
        self.align = rs.align(rs.stream.color)

        # Activar emisor infrarrojo si es posible
        device = self.profile.get_device()
        depth_sensor = device.first_depth_sensor()
        if depth_sensor.supports(rs.option.emitter_enabled):
            depth_sensor.set_option(rs.option.emitter_enabled, 1)

        # Visualizador sin ventana
        self.pcd = o3d.geometry.PointCloud()
        self.render = rendering.OffscreenRenderer(self.image_width, self.image_height)
        self.render.scene.set_background([0, 0, 0, 1])
        self.material = rendering.MaterialRecord()
        self.material.shader = "defaultUnlit"

        self.pc = rs.pointcloud()
        self.first_frame = True

        self.get_logger().info("🚀 Publicación en tiempo real iniciada.")

        try:
            self.process_loop()
        except KeyboardInterrupt:
            self.get_logger().info("🛑 Interrumpido por el usuario.")
        finally:
            self.pipeline.stop()
            #self.render.release()

    def process_loop(self):
        while rclpy.ok():
            # ================== Procesar Nube de Puntos ====================
            frames = self.pipeline.wait_for_frames()
            aligned_frames = self.align.process(frames)
            depth_frame = aligned_frames.get_depth_frame()
            color_frame = aligned_frames.get_color_frame()
            if not depth_frame or not color_frame:
                continue

            self.pc.map_to(color_frame)
            points = self.pc.calculate(depth_frame)
            verts = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)

            color_image = np.asanyarray(color_frame.get_data())
            color_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
            colors = color_rgb.reshape(-1, 3) / 255.0

            min_len = min(len(verts), len(colors))
            verts = verts[:min_len]
            colors = colors[:min_len]

            # Rotar nube
            R = np.array([
                [1, 0,  0],
                [0, -1, 0],
                [0, 0, -1]
            ])
            verts = verts @ R.T

            self.pcd.points = o3d.utility.Vector3dVector(verts)
            self.pcd.colors = o3d.utility.Vector3dVector(colors)

            # ================== Cámaras Virtuales ====================
            depth_stream = depth_frame.profile.as_video_stream_profile()
            color_stream = color_frame.profile.as_video_stream_profile()
            extr = depth_stream.get_extrinsics_to(color_stream)

            extr_matrix = np.eye(4)
            extr_matrix[:3, :3] = np.array(extr.rotation).reshape(3, 3)
            extr_matrix[:3, 3] = np.array(extr.translation)

            camera_position = extr_matrix[:3, 3]
            forward = extr_matrix[:3, 2]
            up = extr_matrix[:3, 1]
            right = np.cross(forward, up)

            lookat = camera_position + forward
            eye_distance = 0.265
            eye_left = camera_position - forward + (right * (eye_distance / 2))
            eye_right = camera_position - forward - (right * (eye_distance / 2))
            lookat_left = eye_left + forward
            lookat_right = eye_right + forward

            self.render.scene.clear_geometry()
            self.render.scene.add_geometry("pcd", self.pcd, self.material)

            self.render.setup_camera(45.0, eye_left, lookat_left, up)
            img_left = self.render.render_to_image()
            img_left_np = np.asarray(img_left)

            self.render.setup_camera(45.0, eye_right, lookat_right, up)
            img_right = self.render.render_to_image()
            img_right_np = np.asarray(img_right)

            # ================== Publicar Imagenes ====================
            msg_left = self.bridge.cv2_to_imgmsg(img_left_np, encoding="rgb8")
            msg_right = self.bridge.cv2_to_imgmsg(img_right_np, encoding="rgb8")
            now = self.get_clock().now().to_msg()
            msg_left.header.stamp = now
            msg_right.header.stamp = now
            self.pub_left.publish(msg_left)
            self.pub_right.publish(msg_right)


def main(args=None):
    rclpy.init(args=args)
    node = StereoVirtualPublisher()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

