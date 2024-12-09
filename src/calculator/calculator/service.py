from example_interfaces.srv import AddTwoInts

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('service')
        # self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.add_service = self.create_service(AddTwoInts, 'add_two_ints', self.add_callback)
        self.subtract_service = self.create_service(AddTwoInts, 'subtract_two_ints', self.subtract_callback)
        self.multiply_service = self.create_service(AddTwoInts, 'multiply_two_ints', self.multiply_callback)
        self.divide_service = self.create_service(AddTwoInts, 'divide_two_ints', self.divide_callback)

    # def add_two_ints_callback(self, request, response):
    #     response.sum = request.a + request.b
    #     self.get_logger().info('add : %d + %d = %d' % (request.a, request.b, response.sum))
    
    def add_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('add : %d + %d = %d' % (request.a, request.b, response.sum))
        return response

    def subtract_callback(self, request, response):
        response.sum = request.a - request.b
        self.get_logger().info('sub : %d - %d = %d' % (request.a, request.b, response.sum))
        return response

    def multiply_callback(self, request, response):
        response.sum = request.a * request.b
        self.get_logger().info('mul : %d * %d = %d' % (request.a, request.b, response.sum))
        return response

    def divide_callback(self, request, response):
        # 나누기 연산 시, 0으로 나누는 것을 방지해야 합니다.
        if request.b == 0:
            response.sum = 0  # 오류를 처리할 기본값 설정
        else:
            response.sum = request.a // request.b
            self.get_logger().info('div : %d / %d = %d' % (request.a, request.b, response.sum))
        return response


def main():
    rclpy.init()

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()