#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class StereoViewer(Node):
    def __init__(self):
        super().__init__('stereo_viewer')

        self.bridge = CvBridge()
        self.sub_left = self.create_subscription(
            Image,
            '/stereo_virtual/left/image_raw',
            self.callback_left,
            10)

        self.sub_right = self.create_subscription(
            Image,
            '/stereo_virtual/right/image_raw',
            self.callback_right,
            10)

        self.left_img = None
        self.right_img = None

    def callback_left(self, msg):
        self.left_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def callback_right(self, msg):
        self.right_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    def show_loop(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.01)

            if self.left_img is not None and self.right_img is not None:
                stereo = cv2.hconcat([self.left_img, self.right_img])
                cv2.imshow("Stereo View", stereo)

                key = cv2.waitKey(1)
                if key == 27:  # ESC para salir
                    break

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)
    node = StereoViewer()
    try:
        node.show_loop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
