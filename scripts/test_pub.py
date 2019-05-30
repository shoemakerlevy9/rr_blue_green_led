#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray

BLUE = 0
GREEN = 1

def talker():
    pub = rospy.Publisher('led_control', Int16MultiArray, queue_size=10)
    rospy.init_node('pub_test', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Int16MultiArray()
        msg.data = [0, 0]
        
        # Edit here to change the LED intesity: values between 0 - 255
        msg.data[BLUE] = 0     # Edit this line to set intensity of Blue light
        msg.data[GREEN] = 255  # Edit this line to set Intesity of Green Light
        
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
