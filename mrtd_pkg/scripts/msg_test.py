#!/usr/bin/env python3

import rospy
import datetime

from mrtd_pkg.msg import Num.msg


def talker():
    pub = rospy.Publisher("custom_msg", type, queue_size=10)
    rospy.init_node('custom taker', anonymous=True)
    r = rospy.Rate(10)


    msg = Num()
    msg.num = 1
    