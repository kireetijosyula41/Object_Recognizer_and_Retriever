#!/usr/bin/env python3

import cv_bridge
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
import moveit_commander

import cv2
import numpy as np
import math
import pandas as pd
import os


class ActionHandler:
    def __init__(self):
        #### Publishers and subscribers
        ## Subscribes to end_position topic, receives (x, y, z) coordinate of arm goal
        # self.end_pos_sub = rospy.Subscriber("end_position", HandPos, self.set_end_pos, queue_size=10)
        ## Interface to control arm movement
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        ## Interface to control arm gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        ## Retrieves current joint angles for inverse kinematics
        self.joint_angle_sub = rospy.Subscriber(
            "joint_states", JointState, self.get_joint_angles
        )

        #### Constants
        ## Minimum and maximum joint angles for the robot arm
        self.q1_range = [-162 * (180 / math.pi), 162 * (180 / math.pi)]
        self.q2_range = [-103 * (180 / math.pi), 90 * (180 / math.pi)]
        self.q3_range = [-53 * (180 / math.pi), 79 * (180 / math.pi)]

        ## Constants for arm dimensions (https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/#dimension)
        self.l1 = 12.8
        self.l2 = 12.4
        self.l3 = 12.6

        ## Storing values for target angles
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0

        ## Storing values for current angles
        self.q1_current = 0
        self.q2_current = 0
        self.q3_current = 0

    def get_joint_angles(self, jointstate):
        self.q1_current = jointstate.position[3]
        self.q2_current = jointstate.position[4]
        self.q3_current = jointstate.position[5]

    ## Perform a 2RIK calculation.
    ## For a 3R manipulator, we can calculate the 2nd and 3rd angles of the arm using planar calculations
    ## Sets q2 and q3 values
    def two_RIK(self, y, x, z):
        print("Attempting to move to angle {0} {1} {2}".format(x, y, z))
        ## Find the position of the second joint
        p3_y = y - self.l2*math.cos(self.q1_current + self.q2_current + self.q3_current)
        p3_z = z - self.l2*math.cos(self.q1_current + self.q2_current + self.q3_current)

        print(p3_y)
        print(p3_z)
        print((p3_y * p3_y + p3_z * p3_z - self.l1 * self.l1 - self.l2 * self.l2))
        print((2 * self.l1 * self.l2))
        
        ## Get angles between joints
        alpha = math.acos((p3_y * p3_y + p3_z * p3_z - self.l1 * self.l1 - self.l2 * self.l2) / (2 * self.l1 * self.l2))
        beta = math.asin((self.l2 * math.sin(alpha)) / (math.sqrt(p3_y*p3_y + p3_z*p3_z)))
        
        ## Find the remaining possible solutions
        q1_a = math.atan(p3_z / p3_y) - beta
        q1_b = math.atan(p3_z / p3_y) + beta

        q2_a = math.pi - alpha
        q2_b = -1*(math.pi - alpha)

        q3_a = (self.q1_current + self.q2_current + self.q3_current) - q1_a - q2_a
        q3_b = (self.q1_current + self.q2_current + self.q3_current) - q1_b - q2_b

        ## Find the best solution
        print("SOL1: q1:{0}, q2:{1}, q3:{2}".format(q1_a, q2_a, q3_a))
        print("SOL2: q1:{0}, q2:{1}, q3:{2}".format(q1_a, q2_a, q3_b))
        print("SOL3: q1:{0}, q2:{1}, q3:{2}".format(q1_b, q2_b, q3_a))
        print("SOL4: q1:{0}, q2:{1}, q3:{2}".format(q1_b, q2_b, q3_b))
        if (q1_a < self.q1_range[0] or q1_a > self.q1_range[1]) or (q2_a < self.q1_range[0] or q1_a > self.q1_range[1]) or (q1_a < self.q1_range[0] or q1_a > self.q1_range[1]):
            self.q1 = q1_b
            self.q2 = q2_b
        else:
            self.q1 = q1_a
            self.q2 = q2_a

        if q3_a < self.q3_range[0] or q3_a > self.q3_range[1]:
            self.q3 = q3_b
        else:
            self.q3 = q3_a


        print("q1: {0}, q2:{1}, q3:{2}".format(self.q1, self.q2, self.q3))

    ## Camera data should be converted to cartesian coordinates (x, y, z).
    ## Based on the limitations of the arm movements, set a limited domain and set
    ## coordinates outside of this domain to the nearest possible point.
    def adjust_coordinates(self):
        pass

    ## Main loop of execution
    def run(self):
        # rate = rospy.Rate(30)
        # while not rospy.is_shutdown():
        #     pass

        print("Current joint angles: {0} : {1} : {2}".format(self.q1_current, self.q2_current, self.q3_current))

        self.two_RIK(0, 0, 5)
        ## Move the arm
        self.move_group_arm.go((0, self.q1, self.q2, self.q3), wait=True)
        self.move_group_arm.stop()
        rospy.sleep(7)

        self.two_RIK(-25, 0, 5)
        ## Move the arm
        self.move_group_arm.go((0, self.q1, self.q2, self.q3), wait=True)
        self.move_group_arm.stop()
        rospy.sleep(7)

        self.two_RIK(0, 0, 5)
        ## Move the arm
        self.move_group_arm.go((0, self.q1, self.q2, self.q3), wait=True)
        self.move_group_arm.stop()
        rospy.sleep(7)


if __name__ == "__main__":
    rospy.init_node("robot_action")
    handler = ActionHandler()
    handler.run()
