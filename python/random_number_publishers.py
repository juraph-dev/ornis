#!/usr/bin/env python
# license removed for brevity
import rospy
import random
from std_msgs.msg import Float32

def talker():
    pub = rospy.Publisher('number', Float32, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        number_val = random.random()
        pub.publish(number_val)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
