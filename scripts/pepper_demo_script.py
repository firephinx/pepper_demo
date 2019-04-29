#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

from __future__ import print_function
import qi
import sys
import time
import almath
import rospy
import math
from pepper_demo.msg import HumanList, Human, Speech
from geometry_msgs.msg import Twist

class PepperDemo(object):

    def __init__(self, app):
        """
        Initialisation of qi framework and event detection.
        """
        super(PepperDemo, self).__init__()
        app.start()
        session = app.session

        self.mode = ""

        self.motion = session.service("ALMotion")
        self.posture = session.service("ALRobotPosture")
        self.tts = session.service("ALTextToSpeech")
        self.tts.setVolume(1.0)

        self.human_sub = rospy.Subscriber("/human_list", HumanList, self.parseHumanList, queue_size=1)
        self.speech_sub = rospy.Subscriber("/speech", Speech, self.parseSpeech, queue_size=1)
        self.names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
        self.angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.fractionMaxSpeed = 0.1     
        
        self.speaking=False
        self.time_last_speech=rospy.get_rostime().secs
        self.time_tolerance=3
        
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)

        self.arm_names = ["RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw"]
        self.arm_angles = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self.head_names = ["HeadYaw", "HeadPitch"]
        self.head_angles = [0, 0]
        self.fractionMaxSpeed = 0.1
        self.right_hand_open = True
        self.left_hand_open = True
        self.useSensors = False
        self.useArmsAndHands = False
        self.useArmsAndHandsWithoutConflicting = True
        self.isFollowing = False
        self.person_id = -1
        self.timeLastHumanListTopicReceived = 0
        self.lastFrameSeen = 0

        self.twist_cmd = Twist()
        self.last_posture = ""
        self.last_command_type = ""

        # Wake up robot
        self.motion.wakeUp()

    def parseHumanList(self, msg):
        self.timeLastHumanListTopicReceived = time.time()
        if(len(msg.human_list) > 0):
            if (self.mode == "move_base_mode" and self.useArmsAndHands and self.useArmsAndHandsWithoutConflicting):
                self.moveAccordingToHandPositions(msg)
            elif (self.mode == "move_arms_mode" and self.useArmsAndHands and self.useArmsAndHandsWithoutConflicting):
                self.mimicArms(msg)
            elif (self.mode == "following_mode" and self.isFollowing):
                self.followPerson(msg)

    def mimicArms(self, msg):
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

                break


    def parseSpeech(self, msg):
        if self.speaking==True or \
           msg.header.stamp.secs-self.time_last_speech > 0 and msg.header.stamp.secs-self.time_last_speech < self.time_tolerance:
            print("Pepper was speaking")
            print(msg.header.stamp.secs-self.time_last_speech)
            return
        print(msg.header.stamp.secs-self.time_last_speech)

        if(msg.transcript != "" and msg.confidence > 0.88):

            if msg.transcript[:3]== "say":
                self.synthesize(msg.transcript[3:])
                return

            if (("raise" in msg.transcript or "put" in msg.transcript) and ("arm" in msg.transcript or "hand" in msg.transcript)) \
                or "mimic" in msg.transcript \
                or "copy" in msg.transcript \
                or "imitate" in msg.transcript \
                or "airplane" in msg.transcript:
                self.mode = "move_arms_mode"
                self.isFollowing = False
            elif "move" in msg.transcript or "turn" in msg.transcript or "go" in msg.transcript or "control" in msg.transcript:
                self.mode = "move_base_mode"
                self.isFollowing = False
            elif "stand" in msg.transcript or "crouch" in msg.transcript:
                self.mode = "posture_mode"
                self.isFollowing = False
            elif "nod" in msg.transcript or "shake" in msg.transcript or "head" in msg.transcript or "look" in msg.transcript:
                self.mode = "head_mode"
            elif "follow" in msg.transcript:
                self.mode = "following_mode"
            elif "stop" in msg.transcript:
                if(self.useArmsAndHandsWithoutConflicting):
                    self.useArmsAndHands = False
                else:
                    self.useArmsAndHandsWithoutConflicting = True

                if self.isFollowing:
                    self.synthesize("I will no longer follow you.")
                    self.isFollowing = False

                twist = Twist()
                self.pub.publish(twist)
                self.motion.stopMove()
                self.posture.stopMove()
                current_angles = self.motion.getAngles(self.arm_names, self.useSensors)
                self.motion.setAngles(self.arm_names,current_angles,self.fractionMaxSpeed)
                self.synthesize("Ok stopping.")
            elif "continue" in msg.transcript:
                self.synthesize("Ok continuing.")
                if (self.mode == "move_arms_mode"):
                    if (self.last_command_type == "speech"):
                        if ("raise" in msg.transcript or "put" in msg.transcript) and "hands" in msg.transcript:
                            self.useArmsAndHands = False
                            self.useArmsAndHandsWithoutConflicting = False
                            self.raiseHands()
                        elif "airplane" in msg.transcript:
                            self.useArmsAndHands = False
                            self.useArmsAndHandsWithoutConflicting = False
                            self.airplane()
                elif (self.mode == "posture_mode"):
                    if (self.last_command_type == "speech"):
                        if (self.last_posture == "stand"):
                            self.useArmsAndHandsWithoutConflicting = False
                            self.posture.applyPosture("StandInit", 0.5)
                        elif (self.last_posture == "crouch"):
                            self.useArmsAndHandsWithoutConflicting = False
                            self.posture.applyPosture("Crouch", 0.5)
                elif (self.mode == "move_base_mode"):
                    if (self.last_command_type == "speech"):
                        self.useArmsAndHandsWithoutConflicting = False
                        self.pub.publish(self.twist_cmd)
                elif (self.mode == "following_mode"):
                    self.isFollowing = True

            if "stop" in msg.transcript and ("hand" in msg.transcript or "arm" in msg.transcript):
                self.useArmsAndHands = False
                self.last_command_type = ""

            if (self.mode == "move_arms_mode"):
                self.handleMoveArmsMode(msg)

            elif (self.mode == "posture_mode"):
                self.handlePostureMode(msg)

            elif (self.mode == "move_base_mode"):
                self.handleMoveBaseMode(msg)

            elif (self.mode == "head_mode"):
                self.handleHeadMode(msg)
                
            elif (self.mode == "following_mode"):
                self.handleFollowingMode(msg)

        print (self.mode)

    def handleMoveArmsMode(self, msg):
        if ("raise" in msg.transcript or "put" in msg.transcript) and "hands" in msg.transcript:
            self.useArmsAndHands = False
            self.useArmsAndHandsWithoutConflicting = False
            self.last_command_type = "speech"
            self.raiseHands()
        elif "airplane" in msg.transcript:
            self.useArmsAndHands = False
            self.useArmsAndHandsWithoutConflicting = False
            self.last_command_type = "speech"
            self.airplane()  
        elif "mimic" in msg.transcript or "copy" in msg.transcript or "imitate" in msg.transcript \
            or ("move" in msg.transcript and ("arm" in msg.transcript or "hand" in msg.transcript)):
            self.useArmsAndHands = True
            self.useArmsAndHandsWithoutConflicting = True
            self.last_command_type = "vision"
            self.synthesize("I will now mimic your arms.")

    def handlePostureMode(self, msg):
        if "stand" in msg.transcript:
            self.useArmsAndHandsWithoutConflicting = False
            self.last_posture = "stand"
            self.last_command_type = "speech"
            self.synthesize("Standing up.")
            self.posture.applyPosture("StandInit", 0.5)

        elif "crouch" in msg.transcript:
            self.useArmsAndHandsWithoutConflicting = False
            self.last_posture = "crouch"
            self.last_command_type = "speech"
            self.synthesize("Am I a ball yet?")
            self.posture.applyPosture("Crouch", 0.5)

    def handleMoveBaseMode(self, msg):
        if "hand" in msg.transcript:
            self.useArmsAndHands = True
            self.useArmsAndHandsWithoutConflicting = True
            self.last_command_type = "vision"
            self.synthesize("I am now following your hand positions.")
            
        if "move" in msg.transcript or "go" in msg.transcript:
            if "forward" in msg.transcript:
                self.twist_cmd.linear.x = 1
                self.useArmsAndHandsWithoutConflicting = False
                self.last_command_type = "speech"
                self.synthesize("Moving Forward.")
            elif "backward" in msg.transcript or "back" in msg.transcript:
                self.twist_cmd.linear.x = -1
                self.useArmsAndHandsWithoutConflicting = False
                self.last_command_type = "speech"
                self.synthesize("Moving Backward.")
            else:
                self.twist_cmd.linear.x = 0

            if "left" in msg.transcript:
                self.twist_cmd.linear.y = 1
                self.useArmsAndHandsWithoutConflicting = False
                self.last_command_type = "speech"
                self.synthesize("Going left.")
            elif "right" in msg.transcript:
                self.twist_cmd.linear.y = -1
                self.useArmsAndHandsWithoutConflicting = False
                self.last_command_type = "speech"
                self.synthesize("Going Right.")
            else:
                self.twist_cmd.linear.y = 0

            self.twist_cmd.angular.z = 0
            self.pub.publish(self.twist_cmd)

        if "turn" in msg.transcript:
            self.twist_cmd.linear.x = 0
            self.twist_cmd.linear.y = 0

            if "clockwise" in msg.transcript:
                if "counter" in msg.transcript:
                    self.twist_cmd.angular.z = 1
                    self.useArmsAndHandsWithoutConflicting = False
                    self.synthesize("Turning counter-clockwise.")
                else:
                    self.twist_cmd.angular.z = -1
                    self.useArmsAndHandsWithoutConflicting = False
                    self.synthesize("Turning clockwise.")
                self.last_command_type = "speech"
                self.pub.publish(self.twist_cmd)
            else:
                self.last_command_type = "speech"
                if "left" in msg.transcript:
                    self.useArmsAndHandsWithoutConflicting = False
                    self.synthesize("Turning left.")
                    self.motion.moveTo(0, 0, 90.0*almath.TO_RAD)
                elif "right" in msg.transcript:
                    self.useArmsAndHandsWithoutConflicting = False
                    self.synthesize("Turning right.")
                    self.motion.moveTo(0, 0, -90.0*almath.TO_RAD)
                elif "around" in msg.transcript:
                    self.useArmsAndHandsWithoutConflicting = False
                    self.synthesize("Turning around.")
                    self.motion.moveTo(0, 0, 180.0*almath.TO_RAD)

    def handleHeadMode(self, msg):
        if "nod" in msg.transcript:
            self.nodHead()
        elif "shake" in msg.transcript:
            self.shakeHead()
        elif "look" in msg.transcript:
            self.lookInDirection(msg)

    def handleFollowingMode(self, msg):
        if "follow" in msg.transcript:
            self.synthesize("I will now start following you.")
            self.isFollowing = True

    def moveAccordingToHandPositions(self, msg):
        if (self.person_id == -1):
            # Select the closest person to follow

            if(len(msg.human_list) > 0):
                human_id = -1
                min_dist = 10.0
                for human in msg.human_list:
                    if human.depth < min_dist:
                        human_id = human.id
                        min_dist = human.depth

                self.person_id = human_id

        if(self.person_id >= 0):

            human_index = -1

            for i in xrange(len(msg.human_list)):
                if msg.human_list[i].id == self.person_id:
                    human_index = i
                    break

            if human_index == -1:
                if msg.image_header.seq - self.lastFrameSeen > 100:
                    self.person_id = -1
            else:

                self.lastFrameSeen = msg.image_header.seq
                human = msg.human_list[human_index]

                if(human.facing_direction == "forward"):
                    if(human.right_hand_vertical_position == "up"):
                        self.twist_cmd.linear.x = 1
                        print("Right hand is up, moving forward")
                    elif(human.left_hand_vertical_position == "up"):
                        self.twist_cmd.linear.x = -1
                        print("Left hand is up, moving backward")
                    else:
                        self.twist_cmd.linear.x = 0

                    if(human.right_hand_horizontal_position == "right" and (human.right_hand_vertical_position == "up" or 
                       human.right_hand_vertical_position == "center") or human.left_hand_horizontal_position == "right" and
                       (human.left_hand_vertical_position == "up" or human.left_hand_vertical_position == "center")):
                        self.twist_cmd.linear.y = 1
                        print("Right/Left hand is right, moving right")
                    elif(human.right_hand_horizontal_position == "left" and (human.right_hand_vertical_position == "up" or 
                         human.right_hand_vertical_position == "center") or human.left_hand_horizontal_position == "left" and
                         (human.left_hand_vertical_position == "up" or human.left_hand_vertical_position == "center")):
                        self.twist_cmd.linear.y = -1
                        print("Right/Left hand is left, moving left")
                    else:
                        self.twist_cmd.linear.y = 0

                    self.twist_cmd.angular.z = 0

                elif(human.facing_direction == "backward"):
                    if(human.right_hand_vertical_position == "up"):
                        self.twist_cmd.linear.x = 1
                        print("Right hand is up, moving forward")
                    elif(human.left_hand_vertical_position == "up"):
                        self.twist_cmd.linear.x = -1
                        print("Left hand is up, moving backward")
                    else:
                        self.twist_cmd.linear.x = 0

                    if(human.right_hand_horizontal_position == "right" and (human.right_hand_vertical_position == "up" or 
                       human.right_hand_vertical_position == "center") or human.left_hand_horizontal_position == "right" and
                       (human.left_hand_vertical_position == "up" or human.left_hand_vertical_position == "center")):
                        self.twist_cmd.linear.y = -1
                        print("Right/Left hand is right, moving right")
                    elif(human.right_hand_horizontal_position == "left" and (human.right_hand_vertical_position == "up" or 
                         human.right_hand_vertical_position == "center") or human.left_hand_horizontal_position == "left" and
                         (human.left_hand_vertical_position == "up" or human.left_hand_vertical_position == "center")):
                        self.twist_cmd.linear.y = 1
                        print("Right/Left hand is left, moving left")
                    else:
                        self.twist_cmd.linear.y = 0

                    self.twist_cmd.angular.z = 0

                elif(human.facing_direction == "right"):
                    if(human.right_hand_vertical_position == "up"):
                        self.twist_cmd.linear.x = 1
                        print("Right hand is up, moving forward")
                    elif(human.left_hand_vertical_position == "up"):
                        self.twist_cmd.linear.x = -1
                        print("Left hand is up, moving backward")
                    else:
                        self.twist_cmd.linear.x = 0

                    self.twist_cmd.linear.y = 0

                    if(human.right_hand_horizontal_position == "forward" and (human.right_hand_vertical_position == "up" or 
                       human.right_hand_vertical_position == "center") or human.left_hand_horizontal_position == "forward" and
                       (human.left_hand_vertical_position == "up" or human.left_hand_vertical_position == "center")):
                        self.twist_cmd.angular.z = -1
                        self.pub.publish(self.twist_cmd)
                        print("Right/Left hand is forward, turning")
                    elif(human.right_hand_horizontal_position == "backward" and (human.right_hand_vertical_position == "up" or 
                         human.right_hand_vertical_position == "center") or human.left_hand_horizontal_position == "backward" and
                         (human.left_hand_vertical_position == "up" or human.left_hand_vertical_position == "center")):
                        self.twist_cmd.angular.z = 1
                        self.pub.publish(self.twist_cmd)
                        print("Right/Left hand is backward, turning")
                    else:
                        self.twist_cmd.angular.z = 0

                elif(human.facing_direction == "left"):
                    if(human.right_hand_vertical_position == "up"):
                        self.twist_cmd.linear.x = 1
                        print("Right hand is up, moving forward")
                    elif(human.left_hand_vertical_position == "up"):
                        self.twist_cmd.linear.x = -1
                        print("Left hand is up, moving backward")
                    else:
                        self.twist_cmd.linear.x = 0

                    self.twist_cmd.linear.y = 0

                    if(human.right_hand_horizontal_position == "forward" and (human.right_hand_vertical_position == "up" or 
                       human.right_hand_vertical_position == "center") or human.left_hand_horizontal_position == "forward" and
                       (human.left_hand_vertical_position == "up" or human.left_hand_vertical_position == "center")):
                        self.twist_cmd.angular.z = 1
                        self.pub.publish(self.twist_cmd)
                        print("Right/Left hand is forward, turning")
                    elif(human.right_hand_horizontal_position == "backward" and (human.right_hand_vertical_position == "up" or 
                         human.right_hand_vertical_position == "center") or human.left_hand_horizontal_position == "backward" and
                         (human.left_hand_vertical_position == "up" or human.left_hand_vertical_position == "center")):
                        self.twist_cmd.angular.z = -1
                        self.pub.publish(self.twist_cmd)
                        print("Right/Left hand is backward, turning")
                    else:
                        self.twist_cmd.angular.z = 0

                self.pub.publish(self.twist_cmd)


    def synthesize(self, speech):
        self.speaking=True
        self.tts.say(speech)
        self.speaking=False
        self.time_last_speech = rospy.get_rostime().secs
        return

    def raiseHands(self):
        self.arm_angles = [-90.0*almath.TO_RAD, 0.0, -90.0*almath.TO_RAD, 0.0, 90.0*almath.TO_RAD, 
                           -90.0*almath.TO_RAD, 0.0, 90.0*almath.TO_RAD, 0.0, -90.0*almath.TO_RAD]
        self.motion.setAngles(self.arm_names,self.arm_angles,self.fractionMaxSpeed)
        self.motion.openHand("RHand")
        self.motion.openHand("LHand")
        self.synthesize("Putting my hands up! Don't shoot me!")

    def airplane(self):
        self.arm_angles = [90.0*almath.TO_RAD, -89.0*almath.TO_RAD, 0.0, 0.0, 90.0*almath.TO_RAD, 
                           90.0*almath.TO_RAD, 89.0*almath.TO_RAD, 0.0, 0.0, -90.0*almath.TO_RAD]
        self.motion.setAngles(self.arm_names,self.arm_angles,self.fractionMaxSpeed)
        self.motion.openHand("RHand")
        self.motion.openHand("LHand")
        self.synthesize("I believe I can fly.")

    def nodHead(self):
        self.head_angles = [0.0, 20.0*almath.TO_RAD]
        self.motion.setAngles(self.head_names,self.head_angles,self.fractionMaxSpeed)
        time.sleep(1.0)
        self.head_angles = [0.0, -30.0*almath.TO_RAD]
        self.motion.setAngles(self.head_names,self.head_angles,self.fractionMaxSpeed)
        time.sleep(1.0)
        self.head_angles = [0.0, -5.0*almath.TO_RAD]
        self.motion.setAngles(self.head_names,self.head_angles,self.fractionMaxSpeed)
        self.synthesize("Okay")

    def shakeHead(self):
        self.head_angles = [60.0*almath.TO_RAD, 0.0]
        self.motion.setAngles(self.head_names,self.head_angles,self.fractionMaxSpeed)
        time.sleep(2.0)
        self.head_angles = [-60.0*almath.TO_RAD, 0.0]
        self.motion.setAngles(self.head_names,self.head_angles,self.fractionMaxSpeed)
        time.sleep(4.0)
        self.head_angles = [0.0, 0.0]
        self.motion.setAngles(self.head_names,self.head_angles,self.fractionMaxSpeed)

    def lookInDirection(self, msg):
        if "up" in msg.transcript:
            self.head_angles = [0.0, -35.0*almath.TO_RAD]
            self.synthesize("Looking up.")
        elif "down" in msg.transcript:
            self.head_angles = [0.0, 35.0*almath.TO_RAD]
            self.synthesize("Looking down.")
        elif "left" in msg.transcript:
            self.head_angles = [90.0*almath.TO_RAD, 0.0]
            self.synthesize("Looking left.")
        elif "right" in msg.transcript:
            self.head_angles = [-90.0*almath.TO_RAD, 0.0]
            self.synthesize("Looking right.")
        elif "backward" in msg.transcript:
            self.head_angles = [119.0*almath.TO_RAD, 0.0]
            self.synthesize("Looking backward.")
        else:
            self.head_angles = [0.0, -5.0*almath.TO_RAD]
            self.synthesize("Looking straight ahead.")
        self.motion.setAngles(self.head_names,self.head_angles,self.fractionMaxSpeed)

    def followPerson(self, msg):
        if (self.person_id == -1):
            # Select the closest person to follow

            if(len(msg.human_list) > 0):
                human_id = -1
                min_dist = 10.0
                for human in msg.human_list:
                    if human.depth < min_dist:
                        human_id = human.id
                        min_dist = human.depth

                self.person_id = human_id

        if(self.person_id >= 0):

            human_index = -1

            for i in xrange(len(msg.human_list)):
                if msg.human_list[i].id == self.person_id:
                    human_index = i
                    break

            if human_index == -1:
                if msg.image_header.seq - self.lastFrameSeen > 100:
                    self.person_id = -1
                self.motion.moveToward(0, 0, 0)
            else:

                self.lastFrameSeen = msg.image_header.seq
                human = msg.human_list[human_index]

                humanPosInImageCoords = human.pos_in_image_coords

                image_x = humanPosInImageCoords.x
                image_y = humanPosInImageCoords.y

                if image_x < 155:
                    self.head_angles[0] = self.head_angles[0] + almath.TO_RAD
                elif image_x > 165:
                    self.head_angles[0] = self.head_angles[0] - almath.TO_RAD

                if image_y < 110:
                    self.head_angles[1] = self.head_angles[1] - almath.TO_RAD
                elif image_y > 130:
                    self.head_angles[1] = self.head_angles[1] + almath.TO_RAD

                self.motion.setAngles(self.head_names,self.head_angles,self.fractionMaxSpeed)

                humanPosInRobotCoords = human.pos_in_robot_coords

                robot_x = humanPosInRobotCoords.x
                robot_z = humanPosInRobotCoords.z

                if(math.isnan(robot_z)):
                    robot_z = human.depth

                if not math.isnan(robot_x):
                    angle = -math.atan2(robot_x, robot_z)
                    if (robot_z > 0.5):
                        self.motion.moveTo(0.3, 0, angle)

if __name__ == "__main__":
    try:
        # Initialize qi framework.
        connection_url = "tcp://" + "CORALPEPPER3.WV.CC.CMU.EDU" + ":" + str(9559)
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
        print ("Can't connect to Naoqi at ip \"" + "CORALPEPPER3.WV.CC.CMU.EDU" + "\" on port " + str(9559) +".\n"
               "Please check your script arguments. Run with -h option for help.")
        sys.exit(1)