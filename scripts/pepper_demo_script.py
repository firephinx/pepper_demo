#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

from __future__ import print_function
import qi
import argparse
import sys
import time
import almath
import rospy
from rcah18_pepper_msgs.msg import SpeechRaw
from pepper_demo.msg import HumanList, Human

class PepperDemo(object):

    def __init__(self, app):
        """
        Initialisation of qi framework and event detection.
        """
        super(PepperDemo, self).__init__()
        app.start()
        session = app.session

        self.motion = session.service("ALMotion")
        self.human_sub = rospy.Subscriber("/human_list", HumanList, self.parseHumanList, queue_size=1)
        self.speech_sub = rospy.Subscriber("/pepper_speech/speech", SpeechRaw, self.parseSpeech, queue_size=1)
        self.names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
        self.angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.fractionMaxSpeed = 0.1

    def parseHumanList(self, msg):
        for human in msg.human_list:
            if(human.facing_direction == "forward"):
                if(human.right_arm_shoulder_angle < 0):
                    self.angles[0] = -90.0*almath.TO_RAD
                    self.angles[1] = (human.right_arm_shoulder_angle + 90.0)*almath.TO_RAD
                    self.angles[2] = 0.0*almath.TO_RAD
                    self.angles[4] = 0.0*almath.TO_RAD
                else:
                    self.angles[0] = 90.0*almath.TO_RAD
                    self.angles[1] = -(human.right_arm_shoulder_angle - 90.0)*almath.TO_RAD
                    self.angles[2] = 90.0*almath.TO_RAD
                    if(human.right_arm_shoulder_angle > 130):
                        self.angles[4] = 90.0*almath.TO_RAD
                    else:
                        self.angles[4] = (human.right_arm_shoulder_angle - 90.0)*2*almath.TO_RAD

                self.angles[3] = (180.0 - human.right_arm_elbow_angle)*almath.TO_RAD

                if(human.left_arm_shoulder_angle < 0):
                    self.angles[5] = -90.0*almath.TO_RAD
                    self.angles[6] = (human.left_arm_shoulder_angle + 90.0)*almath.TO_RAD
                    self.angles[7] = 0.0*almath.TO_RAD
                    self.angles[9] = 0.0*almath.TO_RAD
                else:
                    self.angles[5] = 90.0*almath.TO_RAD
                    self.angles[6] = (90.0 - human.left_arm_shoulder_angle)*almath.TO_RAD
                    self.angles[7] = -90.0*almath.TO_RAD
                    if(human.left_arm_shoulder_angle < 50):
                        self.angles[9] = -90.0*almath.TO_RAD
                    else:
                        self.angles[9] = (human.left_arm_shoulder_angle - 90.0)*2*almath.TO_RAD

                self.angles[8] = -(180.0 - human.left_arm_elbow_angle)*almath.TO_RAD

                self.motion.setAngles(self.names,self.angles,self.fractionMaxSpeed)

    def parseSpeech(self, msg):
        print(msg.transcript)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="CORALPEPPER3.WV.CC.CMU.EDU",
                        help="Robot IP address. On robot or Local Naoqi: use '127.0.0.1'.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Naoqi port number")

    args = parser.parse_args()
    try:
        # Initialize qi framework.
        connection_url = "tcp://" + args.ip + ":" + str(args.port)
        app = qi.Application(["PepperDemo", "--qi-url=" + connection_url])

        try:
            rospy.init_node('pepper_demo')
            pd = PepperDemo(app)
            try:
                rospy.spin()
            except KeyboardInterrupt:
                print("Shutting down")

        except rospy.ROSInterruptException:
            print("Program interrupted before completion", file=sys.stderr)


    except RuntimeError:
        print ("Can't connect to Naoqi at ip \"" + args.ip + "\" on port " + str(args.port) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)