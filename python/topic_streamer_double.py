#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %f", data.data)

def listener():

    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("number", Float32, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
