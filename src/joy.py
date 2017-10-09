#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
import numpy as np


class joyStick(object):

    def __init__(self):

        self._joyPub = rospy.Publisher("vel", Twist, queue_size = 1)
        self._joySub = rospy.Subscriber("/joy", Joy, self.control)
        self.Rate = rospy.Rate(100)

    def control(self, data):

        twist_mag = Twist()

        twist_mag.linear.x = data.axes[0]/10   # Left Joystick side to side
        twist_mag.linear.y = data.axes[1]/10   # Left Joystick up and down
        twist_mag.angular.z = data.axes[2]/10  # Right Joystick up and down
        self._joyPub.publish(twist_mag)
        self.Rate.sleep()

def main():
    rospy.init_node('joy_control')
    joy = joyStick()
    rospy.spin()

if __name__=="__main__":
    main()
