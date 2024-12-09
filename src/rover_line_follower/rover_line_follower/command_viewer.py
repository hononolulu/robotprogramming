#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CommandViewer(Node):
    def __init__(self):
        super().__init__('command_viewer')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.subscription  # 방지: 콜백이 삭제되지 않도록 보관

    def cmd_vel_callback(self, msg):
        # cmd_vel 메시지의 linear와 angular 벡터를 출력
        self.get_logger().info(f"linear : ({msg.linear.x}, {msg.linear.y}, {msg.linear.z}) "
                               f"angular : ({msg.angular.x}, {msg.angular.y}, {msg.angular.z})")

def main(args=None):
    rclpy.init(args=args)
    node = CommandViewer()

    try:
        # 노드 실행
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 노드 종료
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
