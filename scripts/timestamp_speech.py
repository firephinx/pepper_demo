#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

from __future__ import print_function
import argparse
import sys
import time
import math
import rospy
from rcah18_pepper_msgs.msg import SpeechRaw
from pepper_demo.msg import Speech

class TimeStampSpeech(object):

    def __init__(self):

        self.sub = rospy.Subscriber("/pepper_speech/speech", SpeechRaw, self.processSpeech, queue_size=1)
        self.pub = rospy.Publisher("/speech", Speech, queue_size=1)

    def processSpeech(self, msg):
        speech_msg = Speech()
        speech_msg.header.stamp = rospy.Time.now()
        speech_msg.transcript = msg.transcript
        speech_msg.source = msg.source
        speech_msg.confidence = msg.confidence

        self.pub.publish(speech_msg)


if __name__ == "__main__":
    try:
        rospy.init_node('timestamp_speech')
        tss = TimeStampSpeech()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)

