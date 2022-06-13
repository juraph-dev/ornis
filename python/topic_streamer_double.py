#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float32
import math, random

# def callback(data):
#     rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.data)

# def listener():

#     rospy.init_node('listener', anonymous=True)

#     rospy.Subscriber("number", Float32, callback)

#     # spin() simply keeps python from exiting until this node is stopped
#     rospy.spin()

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('sin_wave_pub')

        self.float_publisher_ = self.create_publisher(Float32, 'sin_wave', qos_profile_sensor_data)
        timer_period = 0.1  # Start with 10hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.scale = 1000 * (random.random() - 0.5)

    def timer_callback(self):
         
        # msg_seed = self.scale * math.sin(self.i / 5) 
        msg_seed = self.scale * math.sin(self.i/5) 
        float_msg = Float32()
        float_msg.data = msg_seed
        self.float_publisher_.publish(float_msg)
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
