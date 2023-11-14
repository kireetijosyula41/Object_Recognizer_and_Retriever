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
        ## Publishers and subscribers
        ## Subscribes to end_position topic, receives (x, y, z) coordinate of arm goal
        self.end_pos_sub = rospy.Subscriber("end_position", HandPos, queue_size=10)



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
