import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import UInt64


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('sub')
        self.sub1 = self.create_subscription(
            String,
            'topic1',
            self.topic1_callback,
            10)
        self.sub2 = self.create_subscription(
            UInt64,
            'topic2',
            self.topic2_callback,
            10)
        
        self.msg2 = None

    def topic1_callback(self, msg):
        self.msg1 = msg.data
        if self.msg2 != None:
            self.get_logger().info("I heard: '{} {}'".format(self.msg1, self.msg2))

    def topic2_callback(self, msg):
        self.msg2 = msg.data

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()