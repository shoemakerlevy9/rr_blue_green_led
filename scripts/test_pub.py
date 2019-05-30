#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray
form enum import Enum

color = {
    'blue': 0,
    'green': 1,
    
}

def talker():
    pub = rospy.Publisher('led_control', Int16MultiArray, queue_size=10)
    rospy.init_node('pub_test', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = Int16MultiArray()
        
        # Edit here to change the LED intesity: values between 0 - 255
        msg.data[color.blue] = 0     # Edit this line to set intensity of Blue light
        msg.data[color.green] = 255  # Edit this line to set Intesity of Green Light
        
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
