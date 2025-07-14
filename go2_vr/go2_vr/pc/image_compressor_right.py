#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
import cv2
from cv_bridge import CvBridge
import time

class ImageCompressorNode(Node):
    def __init__(self):
        super().__init__('image_compressor_right')

        # Inicializar el puente de OpenCV
        self.bridge = CvBridge()

        # Suscripción a la imagen derecha
        self.sub = self.create_subscription(Image, '/stereo_virtual/right/image_raw', self.image_callback, 10)

        # Publicador de imagen comprimida
        self.pub = self.create_publisher(CompressedImage, '/stereo_virtual/right/out/compressed', 10)

        # Publicar 15 Hz
        self.last_publish_time = time.time()
        self.publish_interval = 1.0 / 15.0

        self.get_logger().info("ImageCompressorNode (right) iniciado.")

    def image_callback(self, msg):

        # Controlar la frecuencia de publicación
        now = time.time()
        if now - self.last_publish_time < self.publish_interval:
            return

        try:
            # Convertir la imagen a formato OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Comprimir imagen con calidad media (JPEG 70)
            encode_params = [int(cv2.IMWRITE_JPEG_QUALITY), 70]  # calidad media
            success, encoded_image = cv2.imencode('.jpg', cv_image, encode_params)

            if not success:
                self.get_logger().warn("No se pudo codificar la imagen.")
                return

            # Crear mensaje CompressedImage
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header
            compressed_msg.format = "jpeg"
            compressed_msg.data = encoded_image.tobytes()

            # Publicar imagen comprimida
            self.pub.publish(compressed_msg)
            self.last_publish_time = now
        except Exception as e:
            self.get_logger().error(f"Error al convertir/comprimir imagen: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageCompressorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
