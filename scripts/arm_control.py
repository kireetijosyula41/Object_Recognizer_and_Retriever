#!/usr/bin/env python3

import cv_bridge
import rospy
from geometry_msgs.msg import Twist, Vector3
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


        #### Constants
        ## Minimum and maximum r value for the coordinate system
        self.r_min = 0.1
        self.r_max = 0.8

        ## Minimum and maximum theta (angle about x axis) value for the coordinate system
        self.theta_min = 0.0
        self.theta_max = (180.0) * (math.pi / 180) 

        ## Minimum and maximum psi (angle about z axis) value for the coordinate system
        self.theta_min = 0.0
        self.theta_max = (360.0) * (math.pi / 180) 

        ## Constants for arm dimensions (https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/#dimension)
        self.l1 = 12.8
        self.l2 = 12.4
        self.l3 = 12.6

        ## Storing values for target angles
        self.q1 = 0
        self.q2 = 0
        self.q3 = 0

    ## Perform a 2RIK calculation.
    ## For a 3R manipulator, we can calculate the 2nd and 3rd angles of the arm using planar calculations
    ## Sets q2 and q3 values
    def two_RIK(self, x, y, z):
        print("Attempting to move to angle {0} {1} {2}".format(x, y, z))
        ## Find the cosine of the 2nd angle
        ## Move the frame of reference to consider the 2D calculations
        norm = math.sqrt(x*x + y*y)
        cos2 = (norm - (self.l1 * self.l1) - (self.l2 * self.l2)) / (2 * self.l1 * self.l2)
        ## If cos2 is greater than 1, the movement does not have a solution
        if abs(cos2) > 1:
            print('Error: No solution found for target position x:{0}, y:{1}, z:{2}'.format(x, y, z))
            return
        ## If cos2 is 1, q3 = 0
        if cos2 == 1:
            self.q2 = math.atan2(y, x)
            self.q3 = 0
        ## If cos2 is equal to -1, and the target x coordinate is not 0, q3 = pi
        if cos2 == -1 and x != 0:
            self.q2 = math.atan2(y, x)
            self.q3 = math.pi
            return
        ## If cos2 == -1, there are 2 solutions
        if cos2 == -1 and x == 0:
            self.q2 = math.atan()
        
        ## Otherwise, find the remaining possible solutions
        q3_a = math.acos(cos2)
        q3_b = -1 * math.acos(cos2)
        theta = math.atan2(y, x)

        q2_a = theta - math.atan2(self.l2 * math.sin(q3_a), self.l1 + self.l2*math.cos(q3_a))
        q2_b = theta - math.atan2(self.l2 * math.sin(q3_b), self.l1 + self.l2*math.cos(q3_b))

        ## Find the best solution
        ## Arbitrarily select the first angles
        self.q2 = q2_a * (math.pi / 180)
        self.q3 = q3_a * (math.pi / 180)

        ## The first angle value is always the same
        self.q1 = math.atan2(y, x) * (math.pi / 180)

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

        # rate.sleep()
        self.two_RIK(50, 50, 0)
        ## Move the arm
        self.move_group_arm.go([self.q1, self.q2, self.q3, 0], wait=True)
        self.move_group_arm.stop()
        rospy.sleep(10)

        self.two_RIK(0, 0, 0)
        ## Move the arm
        self.move_group_arm.go([self.q1, self.q2, self.q3, 0], wait=True)
        self.move_group_arm.stop()
        rospy.sleep(10)

        self.two_RIK(0, 0, 5)
        ## Move the arm
        self.move_group_arm.go([self.q1, self.q2, self.q3, 0], wait=True)
        self.move_group_arm.stop()
        rospy.sleep(10)


if __name__ == "__main__":
    rospy.init_node("robot_action")
    handler = ActionHandler()
    handler.run()
