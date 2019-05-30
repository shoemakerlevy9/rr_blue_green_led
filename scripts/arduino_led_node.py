#!/usr/bin/env python

import rospy
import serial
from std_msgs.msg import Int16MultiArray

def callback(data):
    led_values = data.data
    # rospy.loginfo(led_values)

    # convert from array (0, 255) to a string of 0,255,
    str_to_send = ''.join(str(e) + ',' for e in led_values)

    # ser.write(b'100, 0')
    ser.write(str_to_send.encode())

def listener():
    rospy.Subscriber("led_control", Int16MultiArray, callback)
    rospy.spin()


if __name__ == '__main__':

    rospy.init_node('arduino_led_node', anonymous=True)

    try:
        port = rospy.get_param('~/port')
    except:
        rospy.logerr('please set param ~/rr_blue_green_led/port...')
    try:
        ser  = serial.Serial(port)
    except:
        rospy.logerr('cannot connect to MultiMoto, check led port')
    try:
        listener()
    except rospy.ROSInterruptException:
        pass
