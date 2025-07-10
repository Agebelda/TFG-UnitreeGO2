import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from unitree_api.msg import Request, Response
from rclpy.qos import QoSProfile, HistoryPolicy, DurabilityPolicy
import json

class CmdVelToRequestBridge(Node):
    def __init__(self):
        super().__init__('cmd_vel_to_request_bridge')

        qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            durability=DurabilityPolicy.VOLATILE
        )

        self.publisher_ = self.create_publisher(Request, '/api/sport/request', qos_profile)
        self.subscription = self.create_subscription(Twist, '/cmd_vel',  self.cmd_vel_callback, qos_profile)

        self.get_logger().info('Nodo /cmd_vel → /api/sport/request activo')

    def cmd_vel_callback(self, msg):
        x = round(msg.linear.x, 2)
        y = round(msg.linear.y, 2)
        z = round(msg.angular.z, 2)

        request_msg = Request()
        request_msg.parameter = json.dumps({
            "x": x,
            "y": y,
            "z": z
        })
        request_msg.header.identity.api_id = 1008

        self.publisher_.publish(request_msg)
        # self.get_logger().info(f'Publicado: x={x}, y={y}, z={z}')

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToRequestBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
