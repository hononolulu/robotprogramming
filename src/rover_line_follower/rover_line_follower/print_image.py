import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from collections import Counter

class ImageSubscriber(Node):
    def __init__(self):
        super().__init__('image_subscriber')

        self.subscription = self.create_subscription(
            Image,
            '/simple_rover/camera/image_raw',
            self.image_callback,
            10
        )
        self.subscription 

        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.imshow('Camera Image', cv_image)
        cv2.waitKey(1)

        # Compute the dominant color
        reshaped_image = cv_image.reshape((-1, 3))  # Flatten the image to a 2D array
        reshaped_image = tuple(map(tuple, reshaped_image))  # Convert to tuple for Counter
        color_counter = Counter(reshaped_image)
        dominant_color, count = color_counter.most_common(1)[0]
        
        # Print the dominant color and its count
        self.get_logger().info(f"Dominant color: {dominant_color}, Count: {count}")

        
        
def main(args=None):
    rclpy.init(args=args)
    image_subscriber = ImageSubscriber()

    try:
        rclpy.spin(image_subscriber)
    except KeyboardInterrupt:
        pass
    finally:
        image_subscriber.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()