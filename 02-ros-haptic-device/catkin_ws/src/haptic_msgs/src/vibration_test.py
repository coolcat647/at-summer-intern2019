#!/usr/bin/python
# -*- coding: UTF-8 -*-

import rospy
import sys
import random
from haptic_msgs.msg import Vibration, VibrationArray

NUM_MOTORS = 3

class VibrationTestNode(object):
    """docstring for VibrationTestNode"""
    def __init__(self):
        self.node_name = rospy.get_name()

        self.pub_vb = rospy.Publisher("change_msg", VibrationArray, queue_size=1)

        self.vb_array = VibrationArray()
        for i in range(NUM_MOTORS):
            vb = Vibration(frequency=0, intensity=0)
            self.vb_array.motors.append(vb)

        rospy.Timer(rospy.Duration(5), self.timer_cb)

    def timer_cb(self, event):
        # Clear motors msg
        for i in range(NUM_MOTORS):
            self.vb_array.motors[i].frequency = 0
            self.vb_array.motors[i].intensity = 0

        # Randomly generate motor ctrl msg
        idx = random.randint(0, NUM_MOTORS-1)
        self.vb_array.motors[idx].frequency = random.randint(1, 5)
        self.vb_array.motors[idx].intensity = random.randint(0, 5)

        self.pub_vb.publish(self.vb_array)


        
        
        
if __name__ == '__main__':
    rospy.init_node("vibration_test_node", anonymous=False)
    node = VibrationTestNode()
    rospy.spin()
