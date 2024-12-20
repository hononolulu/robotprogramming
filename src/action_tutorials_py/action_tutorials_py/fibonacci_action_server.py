#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.import time

import time

from action_tutorials_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def is_prime_num(self, n):
        for i in range(2, n):
            if n % i == 0:
                return False # i로 나누어 떨어지면 소수가 아니므로 False 리턴
        
        return True
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(0, goal_handle.request.goal_number):
            if self.is_prime_num(i+1):
                feedback_msg.partial_sequence.append(i+1)

                if len(feedback_msg.partial_sequence) > 3:
                    self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence[3:]))
                    goal_handle.publish_feedback(feedback_msg)
            
                time.sleep(1)
            
        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    try:
        rclpy.spin(fibonacci_action_server)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()