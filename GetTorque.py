#! /usr/bin/env python2

import numpy as np
import rospy

from std_msgs.msg import Int32

def callback(msg):
    print(msg.effort[0])


sub = rospy.Subscriber('/torobo/joint_states', Int32, callback)