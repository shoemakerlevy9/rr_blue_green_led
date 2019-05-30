#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray

def talker():
    pub = rospy.Publisher('LED_control', Int16MultiArray, queue_size=10)
    rospy.init_node('pub_test', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Int16MultiArray()
        msg.data = [2,200]
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
