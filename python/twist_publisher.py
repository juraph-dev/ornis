#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
import math, random


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('twist_publisher')

        self.twist_publisher_ = self.create_publisher(TwistStamped, 'twist_pub', qos_profile_sensor_data)
        timer_period = 0.1  # Start with 10hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.scale = 1000 * (random.random() - 0.5)

    def timer_callback(self):

        # msg_seed = self.scale * math.sin(self.i / 5)
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.twist.linear.x = 100 * random.random()
        twist_msg.twist.linear.y = 100 * random.random()
        twist_msg.twist.linear.z = 100 * random.random()

        twist_msg.twist.angular.x = random.random()
        twist_msg.twist.angular.y = random.random()
        twist_msg.twist.angular.z = random.random()
        self.twist_publisher_.publish(twist_msg)
        # self.get_logger().info('Publishing: "%s"' % twist_msg)
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
