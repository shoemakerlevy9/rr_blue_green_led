#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int16MultiArray

def callback(data):
    LED_values = data.data
    # rospy.loginfo(LED_values)

    # convert from array (0, 255) to a string of 0,255,
    str_to_send = ''.join(str(e) + ',' for e in LED_values)

    # ser.write(b'100, 0')
    ser.write(str_to_send.encode())

def listener():
    rospy.init_node('LED_control_node', anonymous=True)
    rospy.Subscriber("LED_control", Int16MultiArray, callback)
    rospy.spin()


if __name__ == '__main__':
    ser  = serial.Serial('/dev/ttyACM2')
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
