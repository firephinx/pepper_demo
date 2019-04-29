#! /usr/bin/env python
# -*- encoding: UTF-8 -*-

from __future__ import print_function
import argparse
import sys
import time
import math
import rospy
from openpose_ros_msgs.msg import OpenPoseHumanList
from pepper_demo.msg import HumanList, Human, BoundingBox
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ProcessOpenPoseHumans(object):

    def __init__(self):

        self.nose_position_threshold = 10.0
        self.shoulder_position_threshold = 30.0
        self.hip_position_threshold = 30.0
        self.hip_knee_threshold = 20.0
        self.arm_length_threshold = 2.0
        self.vertical_hand_position_threshold = 20.0
        self.horizontal_hand_position_threshold = 50.0

        self.sub = rospy.Subscriber("/openpose_ros/human_list", OpenPoseHumanList, self.processOpenPoseHumanList, queue_size=1)
        self.depth_image_sub = rospy.Subscriber("/pepper_local_republisher/pepper_robot/camera/depth/image_rect", Image, self.saveDepthImage, queue_size=1)
        self.pub = rospy.Publisher("/human_list", HumanList, queue_size=1)

        self.last_depth_image = None

        self.cv_bridge = CvBridge()

    def saveDepthImage(self, msg):
        self.last_depth_image = self.cv_bridge.imgmsg_to_cv2(msg)

    def processOpenPoseHumanList(self, msg):
        human_list = HumanList()
        human_list.header = msg.header
        human_list.image_header = msg.image_header
        human_list.num_humans = msg.num_humans
        human_id = 0
        for human in msg.human_list:
            new_human = Human()
            new_human.id = human_id
            new_human.num_body_key_points_with_non_zero_prob = human.num_body_key_points_with_non_zero_prob
            new_human.num_face_key_points_with_non_zero_prob = human.num_face_key_points_with_non_zero_prob
            new_human.num_right_hand_key_points_with_non_zero_prob = human.num_right_hand_key_points_with_non_zero_prob
            new_human.num_left_hand_key_points_with_non_zero_prob = human.num_left_hand_key_points_with_non_zero_prob
            new_human.body_bounding_box = self.convertBoundingBox(human.body_bounding_box)
            new_human.face_bounding_box = self.convertBoundingBox(human.face_bounding_box)
            new_human.body_key_points_with_prob = human.body_key_points_with_prob
            new_human.face_key_points_with_prob = human.face_key_points_with_prob
            new_human.right_hand_key_points_with_prob = human.right_hand_key_points_with_prob
            new_human.left_hand_key_points_with_prob = human.left_hand_key_points_with_prob
            new_human.pose = self.determinePose(human)
            new_human.facing_direction = self.determineFacingDirection(human)
            new_human.depth = self.determineDepth(human)

            (right_arm_shoulder_angle, right_arm_elbow_angle, left_arm_shoulder_angle, left_arm_elbow_angle) = self.determineArmAngles(human)
            new_human.right_arm_shoulder_angle = right_arm_shoulder_angle
            new_human.right_arm_elbow_angle = right_arm_elbow_angle
            
            new_human.left_arm_shoulder_angle =left_arm_shoulder_angle
            new_human.left_arm_elbow_angle = left_arm_elbow_angle

            (right_hand_vertical_position, right_hand_horizontal_position, left_hand_vertical_position, left_hand_horizontal_position) = self.determineHandPositions(human, new_human.facing_direction)
            new_human.right_hand_vertical_position = right_hand_vertical_position
            new_human.right_hand_horizontal_position = right_hand_horizontal_position
            new_human.left_hand_vertical_position = left_hand_vertical_position
            new_human.left_hand_horizontal_position = left_hand_horizontal_position

            (right_hand_raised, right_hand_pointing_right, right_hand_pointing_left, left_hand_raised, left_hand_pointing_right, left_hand_pointing_left) = self.determineHighLevelHandCharacteristics(right_hand_vertical_position, right_hand_horizontal_position, left_hand_vertical_position, left_hand_horizontal_position)
            new_human.right_hand_raised = right_hand_raised
            new_human.right_hand_pointing_right = right_hand_pointing_right
            new_human.right_hand_pointing_left = right_hand_pointing_left
            new_human.left_hand_raised = left_hand_raised
            new_human.left_hand_pointing_right = left_hand_pointing_right
            new_human.left_hand_pointing_left = left_hand_pointing_left

            human_list.human_list.append(new_human)

            human_id += 1

        self.pub.publish(human_list)

    def convertBoundingBox(self, bounding_box):
        new_bounding_box = BoundingBox()
        new_bounding_box.x = bounding_box.x
        new_bounding_box.y = bounding_box.y
        new_bounding_box.width = bounding_box.width
        new_bounding_box.height = bounding_box.height

        return new_bounding_box

    def determinePose(self, human):

        right_hip_point_with_prob = human.body_key_points_with_prob[8]
        right_knee_point_with_prob = human.body_key_points_with_prob[9]
        left_hip_point_with_prob = human.body_key_points_with_prob[11]
        left_knee_point_with_prob = human.body_key_points_with_prob[12]

        # If Right Hip and Right Knee are visible
        if(right_hip_point_with_prob.prob > 0.0 and right_knee_point_with_prob.prob > 0.0):
            right_hip_knee_x_diff = abs(right_hip_point_with_prob.x - right_knee_point_with_prob.x)
            if(right_hip_knee_x_diff < self.hip_knee_threshold):
                return "standing"


        # If Left Hip and Left Knee are visible
        if(left_hip_point_with_prob.prob > 0.0 and left_knee_point_with_prob.prob > 0.0):
            left_hip_knee_x_diff = abs(left_hip_point_with_prob.x - left_knee_point_with_prob.x)
            if(left_hip_knee_x_diff < self.hip_knee_threshold):
                return "standing"
        return ""

    def determineDepth(self, human):
        if self.last_depth_image is not None:
            num_depth_points = 0
            total_depth = 0.0

            im_height, im_width = self.last_depth_image.shape

            for i in range(19):
                if(human.body_key_points_with_prob[i].prob > 0.0):
                    x_coord = int(human.body_key_points_with_prob[0].x)
                    y_coord = int(human.body_key_points_with_prob[0].y)
                    if(y_coord > 0) and (y_coord < im_height) and \
                      (x_coord > 0) and (x_coord < im_width) and \
                      not math.isnan(self.last_depth_image[y_coord,x_coord]):
                        total_depth += self.last_depth_image[y_coord,x_coord]
                        num_depth_points += 1

            if(num_depth_points > 0):
                return total_depth / num_depth_points

        return float('nan')
                

    def determineFacingDirection(self, human):
        nose_x = -1.0
        right_shoulder_x = -1.0 
        left_shoulder_x = -1.0 
        right_hip_x = -1.0 
        left_hip_x = -1.0 

        nose_visible = False
        right_shoulder_visible = False
        left_shoulder_visible = False
        right_hip_visible = False
        left_hip_visible = False

        if(human.body_key_points_with_prob[0].prob > 0.0):
            nose_x = human.body_key_points_with_prob[0].x
            nose_visible = True
        if(human.body_key_points_with_prob[2].prob > 0.0):
            right_shoulder_x = human.body_key_points_with_prob[2].x
            right_shoulder_visible = True
        if(human.body_key_points_with_prob[5].prob > 0.0):
            left_shoulder_x = human.body_key_points_with_prob[5].x
            left_shoulder_visible = True
        if(human.body_key_points_with_prob[8].prob > 0.0):
            right_hip_x = human.body_key_points_with_prob[8].x
            right_hip_visible = True
        if(human.body_key_points_with_prob[11].prob > 0.0):
            left_hip_x = human.body_key_points_with_prob[11].x
            left_hip_visible = True

        # Both shoulders are visible
        if(right_shoulder_visible and left_shoulder_visible):
            if(right_shoulder_x < left_shoulder_x - self.shoulder_position_threshold):
                return "forward"
            elif(right_shoulder_x > left_shoulder_x + self.shoulder_position_threshold):
                return "backward"
            else:
                # Nose is visible
                if(nose_visible):
                    average_shoulder_x = (right_shoulder_x + left_shoulder_x) / 2

                    if(nose_x < average_shoulder_x - self.nose_position_threshold):
                        return "left"
                    elif(nose_x > average_shoulder_x + self.nose_position_threshold):
                        return "right"

        # Right shoulder and nose is visible
        if(right_shoulder_visible and nose_visible):
            if(nose_x < right_shoulder_x - self.nose_position_threshold):
                return "left"
            elif(nose_x > right_shoulder_x + self.nose_position_threshold):
                return "right"

        # Left shoulder and nose is visible
        if(left_shoulder_visible and nose_visible):
            if(nose_x < left_shoulder_x - self.nose_position_threshold):
                return "left"
            elif(nose_x > left_shoulder_x + self.nose_position_threshold):
                return "right"
        # Both hips are visible
        if(right_hip_visible and left_hip_visible):
            if(right_hip_x < left_hip_x - self.hip_position_threshold):
                return "forward"
            elif(right_hip_x > left_hip_x + self.hip_position_threshold):
                return "backward"

        return ""

    def determineArmAngles(self, human):
        
        # Right Side
        right_shoulder_x = -1.0
        right_shoulder_y = -1.0
        right_elbow_x = -1.0
        right_elbow_y = -1.0
        right_wrist_x = -1.0
        right_wrist_y = -1.0

        right_shoulder_visible = False
        right_elbow_visible = False
        right_wrist_visible = False

        right_arm_shoulder_angle = float('nan')
        right_arm_elbow_angle = float('nan')

        # Check if right shoulder is visible
        if(human.body_key_points_with_prob[2].prob > 0.0):
            right_shoulder_x = human.body_key_points_with_prob[2].x
            right_shoulder_y = human.body_key_points_with_prob[2].y
            right_shoulder_visible = True
        # Check if right elbow is visible
        if(human.body_key_points_with_prob[3].prob > 0.0):
            right_elbow_x = human.body_key_points_with_prob[3].x
            right_elbow_y = human.body_key_points_with_prob[3].y
            right_elbow_visible = True
        # Check if right wrist is visible
        if(human.body_key_points_with_prob[4].prob > 0.0):
            right_wrist_x = human.body_key_points_with_prob[4].x
            right_wrist_y = human.body_key_points_with_prob[4].y
            right_wrist_visible = True

        # Right shoulder and right wrist are visible
        if(right_shoulder_visible and right_wrist_visible):
            right_arm_angle = math.atan2(right_wrist_y - right_shoulder_y, right_wrist_x - right_shoulder_x) * 180.0 / math.pi
        # Right shoulder and right elbow are visible
        if(right_shoulder_visible and right_elbow_visible):
            right_arm_shoulder_angle = math.atan2(right_elbow_y - right_shoulder_y, right_elbow_x - right_shoulder_x) * 180.0 / math.pi  
        # Right shoulder, right elbow, and right wrist are visible
        if(right_shoulder_visible and right_elbow_visible and right_wrist_visible):
            right_upper_arm_length = math.sqrt((right_shoulder_x - right_elbow_x) * (right_shoulder_x - right_elbow_x) + 
                                                (right_shoulder_y - right_elbow_y) * (right_shoulder_y - right_elbow_y))
            right_forearm_length = math.sqrt((right_wrist_x - right_elbow_x) * (right_wrist_x - right_elbow_x) + 
                                              (right_wrist_y - right_elbow_y) * (right_wrist_y - right_elbow_y))
            right_arm_length = math.sqrt((right_wrist_x - right_shoulder_x) * (right_wrist_x - right_shoulder_x) + 
                                          (right_wrist_y - right_shoulder_y) * (right_wrist_y - right_shoulder_y))

            if(abs(right_upper_arm_length + right_forearm_length - right_arm_length) < self.arm_length_threshold):
                right_arm_elbow_angle = 180.0
            else:
                right_arm_elbow_angle = math.acos(((right_upper_arm_length * right_upper_arm_length) + (right_forearm_length * right_forearm_length)
                                               - (right_arm_length * right_arm_length)) / (2 * right_upper_arm_length * right_forearm_length)) * 180.0 / math.pi


        # Left Side
        left_shoulder_x = -1.0
        left_shoulder_y = -1.0
        left_elbow_x = -1.0
        left_elbow_y = -1.0
        left_wrist_x = -1.0
        left_wrist_y = -1.0

        left_shoulder_visible = False
        left_elbow_visible = False
        left_wrist_visible = False

        left_arm_shoulder_angle = float('nan')
        left_arm_elbow_angle = float('nan')

        # Check if left shoulder is visible
        if(human.body_key_points_with_prob[5].prob > 0.0):
            left_shoulder_x = human.body_key_points_with_prob[5].x
            left_shoulder_y = human.body_key_points_with_prob[5].y
            left_shoulder_visible = True

        # Check if left elbow is visible
        if(human.body_key_points_with_prob[6].prob > 0.0):
            left_elbow_x = human.body_key_points_with_prob[6].x
            left_elbow_y = human.body_key_points_with_prob[6].y
            left_elbow_visible = True

        # Check if left wrist is visible
        if(human.body_key_points_with_prob[7].prob > 0.0):
            left_wrist_x = human.body_key_points_with_prob[7].x
            left_wrist_y = human.body_key_points_with_prob[7].y
            left_wrist_visible = True

        # Left shoulder and left wrist are visible
        if(left_shoulder_visible and left_wrist_visible):
            left_arm_angle = math.atan2(left_wrist_y - left_shoulder_y, left_wrist_x - left_shoulder_x) * 180.0 / math.pi

        # Left shoulder and left elbow are visible
        if(left_shoulder_visible and left_elbow_visible):
            left_arm_shoulder_angle = math.atan2(left_elbow_y - left_shoulder_y, left_elbow_x - left_shoulder_x) * 180.0 / math.pi

        # Left shoulder, left elbow, and left wrist are visible
        if(left_shoulder_visible and left_elbow_visible and left_wrist_visible):
            left_upper_arm_length = math.sqrt((left_shoulder_x - left_elbow_x) * (left_shoulder_x - left_elbow_x) + 
                                                (left_shoulder_y - left_elbow_y) * (left_shoulder_y - left_elbow_y))
            left_forearm_length = math.sqrt((left_wrist_x - left_elbow_x) * (left_wrist_x - left_elbow_x) + 
                                              (left_wrist_y - left_elbow_y) * (left_wrist_y - left_elbow_y))
            left_arm_length = math.sqrt((left_wrist_x - left_shoulder_x) * (left_wrist_x - left_shoulder_x) + 
                                          (left_wrist_y - left_shoulder_y) * (left_wrist_y - left_shoulder_y))
            
            if(abs(left_upper_arm_length + left_forearm_length - left_arm_length) < self.arm_length_threshold):
                left_arm_elbow_angle = 180.0
            else:
                left_arm_elbow_angle = math.acos(((left_upper_arm_length * left_upper_arm_length) + (left_forearm_length * left_forearm_length)
                                               - (left_arm_length * left_arm_length)) / (2 * left_upper_arm_length * left_forearm_length)) * 180.0 / math.pi

        return (right_arm_shoulder_angle, right_arm_elbow_angle, left_arm_shoulder_angle, left_arm_elbow_angle)

    def determineHandPositions(self, human, facing_direction):
        # Right Side
        # Right shoulder and right wrist are visible
        if(human.body_key_points_with_prob[2].prob > 0.0 and human.body_key_points_with_prob[4].prob > 0.0):
            right_shoulder_x = human.body_key_points_with_prob[2].x
            right_shoulder_y = human.body_key_points_with_prob[2].y
            right_wrist_x = human.body_key_points_with_prob[4].x
            right_wrist_y = human.body_key_points_with_prob[4].y

            right_hand_vertical_position = self.determineVerticalHandPosition(right_wrist_y, right_shoulder_y)
            right_hand_horizontal_position = self.determineHorizontalHandPosition(facing_direction, right_wrist_x, right_shoulder_x)
        else:
            right_hand_vertical_position = ""
            right_hand_horizontal_position = ""

        # Left Side
        # Left shoulder and left wrist are visible
        if(human.body_key_points_with_prob[5].prob > 0.0 and human.body_key_points_with_prob[7].prob > 0.0):
            left_shoulder_x = human.body_key_points_with_prob[5].x
            left_shoulder_y = human.body_key_points_with_prob[5].y
            left_wrist_x = human.body_key_points_with_prob[7].x
            left_wrist_y = human.body_key_points_with_prob[7].y
            
            left_hand_vertical_position = self.determineVerticalHandPosition(left_wrist_y, left_shoulder_y)
            left_hand_horizontal_position = self.determineHorizontalHandPosition(facing_direction, left_wrist_x, left_shoulder_x)
        else:
            left_hand_vertical_position = ""
            left_hand_horizontal_position = ""

        return (right_hand_vertical_position, right_hand_horizontal_position, left_hand_vertical_position, left_hand_horizontal_position)

    def determineVerticalHandPosition(self, wrist_y, shoulder_y):
        vertical_hand_position = ""

        if(wrist_y < shoulder_y - self.vertical_hand_position_threshold):
            vertical_hand_position = "up"
        elif(wrist_y > shoulder_y + self.vertical_hand_position_threshold):
            vertical_hand_position = "down"
        else:
            vertical_hand_position = "center"

        return vertical_hand_position

    def determineHorizontalHandPosition(self, facing_direction, wrist_x, shoulder_x):
        horizontal_hand_position = ""

        if(facing_direction == "forward"):
            if(wrist_x < shoulder_x - self.horizontal_hand_position_threshold):
                horizontal_hand_position = "right"
            elif(wrist_x > shoulder_x + self.horizontal_hand_position_threshold):
                horizontal_hand_position = "left"
            else:
                horizontal_hand_position = "center"
        elif(facing_direction == "backward"):
            if(wrist_x < shoulder_x - self.horizontal_hand_position_threshold):
                horizontal_hand_position = "left"
            elif(wrist_x > shoulder_x + self.horizontal_hand_position_threshold):
                horizontal_hand_position = "right"
            else:
                horizontal_hand_position = "center"
        elif(facing_direction == "right"):
            if(wrist_x < shoulder_x - self.horizontal_hand_position_threshold):
                horizontal_hand_position = "backward"
            elif(wrist_x > shoulder_x + self.horizontal_hand_position_threshold):
                horizontal_hand_position = "forward"
            else:
                horizontal_hand_position = "center"
        elif(facing_direction == "left"):
            if(wrist_x < shoulder_x - self.horizontal_hand_position_threshold):
                horizontal_hand_position = "forward"
            elif(wrist_x > shoulder_x + self.horizontal_hand_position_threshold):
                horizontal_hand_position = "backward"
            else:
                horizontal_hand_position = "center"

        return horizontal_hand_position

    def determineHighLevelHandCharacteristics(self, right_hand_vertical_position, right_hand_horizontal_position, left_hand_vertical_position, left_hand_horizontal_position):
        if(right_hand_vertical_position == "up"):
            right_hand_raised = True
        else:
            right_hand_raised = False

        if(right_hand_vertical_position == "center" and right_hand_horizontal_position == "right"):
            right_hand_pointing_right = True
        else:
            right_hand_pointing_right = False

        if(right_hand_vertical_position == "center" and right_hand_horizontal_position == "left"):
            right_hand_pointing_left = True
        else:
            right_hand_pointing_left = False

        if(left_hand_vertical_position == "up"):
            left_hand_raised = True
        else:
            left_hand_raised = False

        if(left_hand_vertical_position == "center" and left_hand_horizontal_position == "right"):
            left_hand_pointing_right = True
        else:
            left_hand_pointing_right = False

        if(left_hand_vertical_position == "center" and left_hand_horizontal_position == "left"):
            left_hand_pointing_left = True
        else:
            left_hand_pointing_left = False

        return (right_hand_raised, right_hand_pointing_right, right_hand_pointing_left, left_hand_raised, left_hand_pointing_right, left_hand_pointing_left)

if __name__ == "__main__":
    try:
        rospy.init_node('process_openpose_humans')
        poph = ProcessOpenPoseHumans()
        try:
            rospy.spin()
        except KeyboardInterrupt:
            print("Shutting down")

    except rospy.ROSInterruptException:
        print("Program interrupted before completion", file=sys.stderr)

