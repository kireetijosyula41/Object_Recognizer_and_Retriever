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
        self.end_pos_sub = rospy.Subscriber("end_position", HandPos, self.set_end_pos, queue_size=10)
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
        self.l1 = 0.130
        self.l2 = 0.124
        self.l3 = 0.126


    ## Camera data should be converted to cartesian coordinates (x, y, z).
    ## Based on the limitations of the arm movements, set a limited domain and set
    ## coordinates outside of this domain to the nearest possible point.
    def adjust_coordinates(self):
        pass

    ## Main loop of execution
    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            pass

        rate.sleep()


if __name__ == "__main__":
    rospy.init_node("robot_action")
    handler = ActionHandler()
    handler.run()
