#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import numpy as np

MOTOR_MAX = 400

def sinewave():

    pub = rospy.Publisher('auto_rover/cmd_vel', Twist, queue_size=1)
    rospy.init_node('pololu_motor_controller_testing', anonymous=True)
    vel = Twist()

    samp_rate = rospy.get_param('sample_rate', 10)
    freq = rospy.get_param('frequency', 0.2)
    rate = rospy.Rate(samp_rate)

    x = np.arange(0, 1/freq * samp_rate, 1./samp_rate)
    linear = 400 * np.sin(2 * np.pi * freq * x)
    linear = np.round(linear).astype(np.int16)

    # Create sinewave values
    idx = 0
    while not rospy.is_shutdown():
        vel.linear.x = linear[idx % x.size]
        pub.publish(vel)
        idx += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        sinewave()
    except rospy.ROSInterruptException:
        rospy.loginfo('Caught Ctrl-C')
