#!/usr/bin/env python3

import cv_bridge
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
from Object_Recognizer_and_Retriever.msg import HandPos
import moveit_commander

import cv2
import numpy as np
import math

import os


class ActionHandler:
    def __init__(self):
        #### Publishers and subscribers
        ## Subscribes to end_position topic, receives (x, y, z) coordinate of arm goal
        # self.end_pos_sub = rospy.Subscriber("hand_control_topic", HandPos, self.set_end_pos, queue_size=10)
        ## Interface to control arm movement
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        ## Interface to control arm gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        # ## Retrieves current joint angles for inverse kinematics
        # self.joint_angle_sub = rospy.Subscriber(
        #     "joint_states", JointState, self.get_joint_angles
        # )
        ## Retrieves hand data from camera
        self.joint_angle_sub = rospy.Subscriber(
            "hand_control_topic", HandPos, self.get_hand_pos
        )

        #### Constants and variables
        ## Minimum and maximum joint angles for the robot arm
        self.q1_range = [-162 * (math.pi / 180), 162 * (math.pi / 180)]
        self.q2_range = [-102 * (math.pi / 180), 83 * (math.pi / 180)]
        self.q3_range = [-54 * (math.pi / 180), 79 * (math.pi / 180)]
        self.tilt_range = [-90, 90]

        self.simulation = False

        ## Constants for arm dimensions (https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/#dimension)
        self.l1 = 12.8
        self.l2 = 12.4 
        self.l3 = 12.6

        ## Storing values for target angles
        self.q1 = 0
        self.q2 = 0
        self.q3 = -102.0 * (math.pi / 180)

        ## Storing values for current angles
        self.q1_current = 0
        self.q2_current = 0
        self.q3_current = 0

        ## Storing values for target position
        self.target_x = 0
        self.target_y = 0
        self.target_z = 5
        self.tilt_angle = 0

    def get_hand_pos(self, hand):
        self.target_x = hand.point.x / 10.0
        self.target_y = hand.point.y / 10.0
        self.target_z = hand.point.z / 10.0

        self.tilt_angle = hand.tilt_angle
        print("Received tilt angle: {0}".format(hand.tilt_angle))



    def get_joint_angles(self, jointstate):
        # print(jointstate.position)
        if len(jointstate.position) >= 6:
            self.q1_current = jointstate.position[3]
            self.q2_current = jointstate.position[4]
            self.q3_current = jointstate.position[5]

    ## Perform a 2RIK calculation.
    ## For a 3R manipulator, we can calculate the 2nd and 3rd angles of the arm using planar calculations
    ## Sets q2 and q3 values
    def two_RIK(self, x, y, z):
        # dist = math.sqrt(z*z + y*y)
        print("Attempting to move to position {0} {1} {2}".format(x, y, z))
        # print("From joint angles {0} {1} {2}".format(self.q1_current, self.q2_current, self.q3_current))
        # print("Term1:{0}".format((y * y + z * z - self.l1 * self.l1 - self.l2 * self.l2)))
        # print("Term2:{0}".format((2 * self.l1 * self.l2)))
        # ## Find the angle q2
        # term1 = (dist - (self.l1 * self.l1) - (self.l2 * self.l2))
        # term2 = (2 * self.l1 * self.l2)
        # q2_a = math.acos((dist - (self.l1 * self.l1) - (self.l2 * self.l2)) / (2 * self.l1 * self.l2))

        # beta = math.atan( (self.l2 * math.sin(q2_a)) / (self.l1 + self.l2 * math.cos(q2_a)) )
        # if z == 0:
        #     q1_a = math.pi / 4 - (math.atan(z / y) - math.atan( (self.l2 * math.sin(q2_a)) / (self.l1 + self.l2 * math.cos(q2_a)) ))
        # else:
        #     q1_a = math.atan(y / z) - math.atan( (self.l2 * math.sin(q2_a)) / (self.l1 + self.l2 * math.cos(q2_a)) )

        # q2_b =  math.acos((-1 * dist + (self.l1 * self.l1) + (self.l2 * self.l2)) / (2 * self.l1 * self.l2))
        # if z == 0:
        #     q1_b = 0
        # else:
        #     q1_b = math.atan(y / z) + math.atan((self.l2 * math.sin(q2_b) / (self.l1 + self.l2 * math.cos(q2_b))))
        # q3_a = 0
        # q3_b = 0

        # ## Find the best solution
        # print("SOL1: {0}".format(self.rads3([q1_a, q2_a, q3_a])))
        # print("SOL2: {0}".format(self.rads3([q1_a, q2_a, q3_b])))
        # print("SOL3: {0}".format(self.rads3([q1_b, q2_b, q3_a])))
        # print("SOL4: {0}".format(self.rads3([q1_b, q2_b, q3_b])))

        # print("RANGES: {0}".format(self.q3_range))
        # if q1_a > self.q2_range[1]:
        #     q1_a = self.q2_range[1]
        # if q1_a < self.q2_range[0]:
        #     q1_a = self.q2_range[0]
        # q2_a = (math.pi / 4) - q2_a
        # print('q2a:{0}'.format(q2_a))
        # if q2_a > self.q3_range[1]:
        #     q2_a = self.q3_range[1]
        # if q2_a < self.q3_range[0]:
        #     q2_a = self.q3_range[0]

        # if z != 0:
        #     self.q1 = (math.pi / 4) - abs(q1_a)
        # else:
        #     self.q1 = q1_a
        # self.q2 = -q2_b
        # self.q3 = -100 * (math.pi / 180)
        link1_length = self.l1
        link2_length = self.l2

        # Calculate the distance between the end effector and the origin
        distance = math.sqrt(z**2 + y**2)

        # Check if the target position is reachable
        if distance > link1_length + link2_length or distance < abs(link1_length - link2_length):
            print("Target position is not reachable.")
            return

        # Calculate the angle between the line connecting the joints and the x-axis
        alpha = math.atan2(z, y)

        # Law of cosines to find the angle at the second joint
        cos_theta2 = (link1_length**2 + link2_length**2 - distance**2) / (2 * link1_length * link2_length)
        sin_theta2 = math.sqrt(1 - cos_theta2**2)
        theta2 = math.atan2(sin_theta2, cos_theta2)

        # Law of sines to find the angle at the first joint
        sin_theta1 = (link2_length * math.sin(theta2)) / distance
        cos_theta1 = (y * (link1_length + link2_length * math.cos(theta2)) + z * link2_length * math.sin(theta2)) / (distance * (link1_length + link2_length * math.cos(theta2)))
        theta1 = math.atan2(z / distance, y / distance) - math.atan2(sin_theta1, cos_theta1)
        print("q1: {0}, q2:{1}, q3:{2}".format(theta1 * (180 / math.pi), theta2* (180 / math.pi), self.q3* (180 / math.pi)))

        theta1 = -theta1
        theta2 = -theta2

        print("q1: {0}, q2:{1}, q3:{2}".format(theta1 * (180 / math.pi), theta2* (180 / math.pi), self.q3* (180 / math.pi)))


        if theta1 < self.q2_range[0]:
            theta1 = self.q2_range[0]
        if theta1 > self.q2_range[1]:
            theta1 = self.q2_range[1]
        if theta2 < self.q3_range[0]:
            theta2 = self.q3_range[0]
        if theta2 > self.q3_range[1]:
            theta2 = self.q3_range[1]

        self.q1 = theta1
        self.q2 = theta2

        print("q1: {0}, q2:{1}, q3:{2}".format(self.q1 * (180 / math.pi), self.q2* (180 / math.pi), self.q3* (180 / math.pi)))

    def set_tilt(self, tilt_angle):
        if tilt_angle < self.tilt_range[0]:
            self.tilt_angle = self.tilt_range[0]
        if tilt_angle > self.tilt_range[1]:
            self.tilt_angle = self.tilt_range[1]

        print("Calculated tilt angle (deg): {0}".format(self.tilt_angle))
        self.tilt_angle = self.tilt_angle * (math.pi / 180)
        print("Calculated tilt angle (rad): {0}".format(self.tilt_angle))

    ## Camera data should be converted to cartesian coordinates (x, y, z).
    ## Based on the limitations of the arm movements, set a limited domain and set
    ## coordinates outside of this domain to the nearest possible point.
    def adjust_coordinates(self):
        pass

    def rads3(self, angles):
        deg1 = angles[0] * (180 / math.pi)
        deg2 = angles[1] * (180 / math.pi)
        deg3 = angles[2] * (180 / math.pi)
        return [deg1, deg2, deg3]

    ## Main loop of execution
    def run(self):
        if not self.simulation:
            while not rospy.is_shutdown():
                self.two_RIK(self.target_x, self.target_y, self.target_z)
                self.set_tilt(self.tilt_angle)
                ## Raise the arm
                self.move_group_arm.go((0, self.q1, self.q2, self.tilt_angle), wait=False)
                self.move_group_arm.stop()
                rospy.sleep(1)
        # print("Current joint angles: {0}".format(self.rads3([self.q1_current, self.q2_current, self.q3_current])))
        else:
            ## Simulate hand camera data
            self.two_RIK(0, 5, 1)
            self.move_group_arm.go((0, self.q1, self.q2, -100 * (math.pi/180)), wait=True)
            self.move_group_arm.stop()
            rospy.sleep(0.1)

            self.two_RIK(0, 5, 4)
            self.move_group_arm.go((0, self.q1, self.q2, -100 * (math.pi/180)), wait=True)
            self.move_group_arm.stop()
            rospy.sleep(0.1)

            self.two_RIK(0, 5, 3)
            self.move_group_arm.go([0, self.q1, self.q2, -100 * (math.pi/180)], wait=True)
            self.move_group_arm.stop()
            rospy.sleep(0.1)

            self.two_RIK(0, 6, 3)
            self.move_group_arm.go((0, self.q1, self.q2, -100 * (math.pi/180)), wait=True)
            self.move_group_arm.stop()
            rospy.sleep(0.1)

            self.two_RIK(0, 4, 5)
            self.move_group_arm.go((0, self.q1, self.q2, -100 * (math.pi/180)), wait=True)
            self.move_group_arm.stop()
            rospy.sleep(0.1)

            self.two_RIK(0, 5, 5)
            self.move_group_arm.go((0, self.q1, self.q2, -100 * (math.pi/180)), wait=True)
            self.move_group_arm.stop()
            rospy.sleep(0.1)

            self.two_RIK(0, 6, 5)
            self.move_group_arm.go((0, self.q1, self.q2, -100 * (math.pi/180)), wait=True)
            self.move_group_arm.stop()
            rospy.sleep(0.1)

            self.two_RIK(0, 7, 4)
            self.move_group_arm.go((0, self.q1, self.q2, -100 * (math.pi/180)), wait=True)
            self.move_group_arm.stop()
            rospy.sleep(0.1)

            self.two_RIK(0, 8, 3)
            self.move_group_arm.go((0, self.q1, self.q2, -100 * (math.pi/180)), wait=True)
            self.move_group_arm.stop()
            rospy.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("robot_action")
    handler = ActionHandler()
    handler.run()
