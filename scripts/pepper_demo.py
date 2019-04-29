#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

from __future__ import print_function
import argparse
import sys
import time
import rospy
from pepper_demo.msg import HumanList, Human

class PepperDemo(object):

    def __init__(self):
        self.human_sub = rospy.Subscriber("/human_list", HumanList, self.parseHumanList, queue_size=1)
        

    def parseHumanList(self, msg):

        

if __name__ == "__main__":
    try:
        rospy.init_node('table_service_demo')
        tsd = PepperDemo()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)