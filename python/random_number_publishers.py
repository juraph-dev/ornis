#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

import random

from std_msgs.msg import Float32, Int32, String
from example_interfaces.msg import MultiFormat


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('random_number_pub')

        self.float_publisher_ = self.create_publisher(Float32, 'random_float', qos_profile_sensor_data)
        self.int_publisher_ = self.create_publisher(Int32, 'random_int', qos_profile_sensor_data)
        self.string_publisher_ = self.create_publisher(String, 'random_string', qos_profile_sensor_data)
        self.multi_publisher_ = self.create_publisher(MultiFormat, 'multi_format', qos_profile_sensor_data)
        timer_period = 0.2  # Start with 10hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg_seed = 100 * random.random()
        float_msg = Float32()
        float_msg.data = msg_seed
        int_msg = Int32()
        int_msg.data = int(msg_seed * 100)
        string_msg = String()
        string_msg.data = "Message no: " + str(self.i) + ", val: " + str(int_msg) + "  \ o.0 /"
        multi_msg = MultiFormat()
        multi_msg.float_data.data = msg_seed
        multi_msg.int_data.data = int_msg.data
        multi_msg.string_data.data = string_msg.data
        self.float_publisher_.publish(float_msg)
        self.int_publisher_.publish(int_msg)
        self.string_publisher_.publish(string_msg)
        self.multi_publisher_.publish(multi_msg)
        self.get_logger().info('Publishing: "%s"' % float_msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
