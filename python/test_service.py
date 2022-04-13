#!/usr/bin/env python3
from example_interfaces.srv import AddTwoInts, AddTwoFloats, AddTwoStrings

import rclpy
from rclpy.node import Node


class MinimalService(Node):

    def __init__(self):
        super().__init__('minimal_service')
        self.int_srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)
        self.floats_srv = self.create_service(AddTwoFloats, 'add_two_floats', self.add_two_floats_callback)
        self.string_srv = self.create_service(AddTwoStrings, 'add_two_strings', self.add_two_strings_callback)

    def add_two_ints_callback(self, request, response):
        self.get_logger().info('[Int] Incoming request\na: %d b: %d' % (request.a, request.b))
        response.sum = request.a + request.b
        return response

    def add_two_floats_callback(self, request, response):
        self.get_logger().info('[Float] Incoming request\na: %f b: %f' % (request.a, request.b))
        response.sum = request.a + request.b
        return response

    def add_two_strings_callback(self, request, response):
        print('[String] Incoming request')
        print(request.a)
        print(request.b)
        response.sum = request.a.data + request.b.data
        return response


def main(args=None):
    rclpy.init(args=args)

    minimal_service = MinimalService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
