#!/usr/bin/env python3

import cv_bridge
import rospy
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import String
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
        self.move_group_arm.set_max_velocity_scaling_factor(0.3)
        self.move_group_arm.set_max_acceleration_scaling_factor(0.3)
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

        ## Retrieves state from robot driver
        self.state_sub = rospy.Subscriber(
            "robot_state", String, self.act_on_state
        )

        #### Constants and variables
        self.simulation = False

        ## Constants for arm dimensions (https://emanual.robotis.com/docs/en/platform/openmanipulator_x/specification/#dimension)
        self.l1 = 12.8
        self.l2 = 12.4 
        self.l3 = 12.6

        ## Maximum distance the arm can reach (law of cosines)
        ## Furthest possible forward reach is at angles q1=83, q2=-54
        self.max_dist = math.sqrt(self.l1**2 + self.l2**2 - (2 * self.l1 * self.l2 * math.cos(np.radians(144))))
        self.min_dist = math.sqrt(self.l1**2 + self.l2**2 - (2 * self.l1 * self.l2 * math.cos(np.radians(11))))
        ## This results in a minimum z value of 10.05
        self.z_min = 10.05

        ## Boolean for whether to take IK actions
        self.arm_mode = True

        ## Storing values for target angles
        self.q0 = 0
        self.q1 = 0
        self.q2 = 0
        self.q3 = -102.0 * (math.pi / 180)

        ## Angle ranges
        self.q0_range = [np.radians(-162), np.radians(162)]
        self.q1_range = [np.radians(-103), np.radians(83)]
        self.q2_range = [np.radians(-54), np.radians(79)]
        self.q3_range = [np.radians(-103), np.radians(117)]
        self.grip_range = [-0.010, 0.019]
        self.z_range = [0.0, self.max_dist]

        ## Storing values for current angles
        self.q1_current = 0
        self.q2_current = 0
        self.q3_current = 0

        ## Storing values for target position
        self.target_x = 0
        self.target_y = 10
        self.target_z = 5
        self.tilt_angle = 0
        self.true_z = 0
        self.gripper_target = -0.01

    def get_hand_pos(self, hand):
        # print("Received point:{0}".format(hand.point))
        self.target_x = hand.point.x
        self.target_y = hand.point.y
        self.target_z = hand.point.z
        self.true_z = hand.point.z

        self.gripper_target = max(-0.01, self.quantize([0.03, 0.09], self.grip_range, hand.gripper_value * 2))
        print("GRIPGRIP:{0}".format(self.gripper_target))

        
        self.tilt_angle = np.radians(hand.tilt_angle)
        # print("Received tilt angle: {0}".format(hand.tilt_angle))

    def act_on_state(self, strn):
       
        if True:#str(strn.data) == str("Start"):
            self.arm_mode = True
            print("ARM MODE IS TRUE")
        else:
            self.arm_mode = False
            print("ARM MODE IS FALSE")


    ## Perform a 2RIK calculation.
    ## For a 3R manipulator, we can calculate the 2nd and 3rd angles of the arm using planar calculations
    ## Sets q2 and q3 values
    def two_RIK(self, x, y, z):
        # Calculate the distance between the end effector and the origin
        distance = math.sqrt(z**2 + y**2)
        ## Adjust coordinates
        adjusted = self.adjust_coordinates(x, y, z)
        y = adjusted[0]
        z = adjusted[1]
        print(adjusted)
        # dist = math.sqrt(z*z + y*y)
        print("Attempting to move to position {0} {1} {2}".format(x, y, z))

        y_true = y
        z_true = z
        y = abs(y)
        z = abs(z)

        ## Calculate the angle to turn the arm
        if x == 0:
            self.q0 = 0.0
        elif x > 0:
            self.q0 = -((math.pi / 2) - math.atan(y / x))
            # else:
            #     self.q0 = -((math.pi / 2) + math.atan(y / x))
        else:
            self.q0 = math.pi - ((math.pi / 2) - math.atan(y / x))
            # else:
            #     self.q0 = (math.pi / 2) + math.atan(y / x)

        print("from x={0} y={1} z={3}, angle = {2}".format(x, y_true, np.degrees(self.q0), z_true))

        numerator = ( -(self.l1**2) + -(self.l2**2) + (y*y) + (z*z) )
        denominator = (2 * self.l1 * self.l2)

        if abs(numerator) >= abs(denominator):
            return

        q2a = math.acos(( -(self.l1**2) + -(self.l2**2) + (y*y) + (z*z) ) / (2 * self.l1 * self.l2))
        q2b = math.acos(( (self.l1**2) + (self.l2**2) + -(y*y) + -(z*z) ) / (2 * self.l1 * self.l2))

        if y == 0:
            term1 = 0
        else:
            term1 = math.atan(z/y)
        # print(math.atan(z/y))
        # print(math.atan(( self.l2 * math.sin(q2a) ) / (self.l1 + self.l2 * math.cos(q2a))))
        q1a = term1 - math.atan(( self.l2 * math.sin(q2a) ) / (self.l1 + self.l2 * math.cos(q2a)))
        q1b = term1 - math.atan(( self.l2 * math.sin(q2b) ) / (self.l1 + self.l2 * math.cos(q2b)))


        ## Adjust angles based on quadrant
        ## Q1
        if y_true > 0.0 and z_true >= 0.0:
            # print("Q1")
            q1a = (math.pi / 2) - q1a
            q2a = -q2a#-((math.pi / 2) - q2a)
            q1b = (math.pi / 2) - q1b
            q2b = -((math.pi / 2) - q2b)
        elif y_true == 0:
            # print("Q1/Q2 (y=0)")
            q1a = q1a
            q2a = -q2a#-((math.pi / 2) - q2a)
            q1b = q1b
            q2b = -((math.pi / 2) - q2b)
        ## Q2. Mirror q1
        elif y_true < 0.0 and z_true > 0.0:
            # print("Q2")
            q1a = -((math.pi / 2) - q1a)
            q2a = (-((math.pi / 2) - q2a))
            q1b = -((math.pi / 2) - q1b)
            q2b = (-((math.pi / 2) - q2b))

        ## Ensure all angles are within tolerances
        self.q1 = self.clamp(self.q1_range, q1a)
        # print(self.clamp(self.q1_range, q1a))
        # print(self.q1)
        self.q2 = self.clamp(self.q2_range, q2a)

        # print("q0: {3}, q1b: {0}, q2b:{1}, q3:{2}".format(np.degrees(q1b), np.degrees(q2b), self.q3* (180 / math.pi), self.q0))
        print("q0: {3}, q1a: {0}, q2a:{1}, q3:{2}".format(math.degrees(self.q1), self.q2* (180 / math.pi), self.q3* (180 / math.pi), np.degrees(self.q0)))

    def clamp(self, r, value):
        if value < r[0]:
            return r[0]
        elif value > r[1]:
            return r[1]

        return value

    ## Maps a value in interval1 to a value in interval2
    def quantize(self, interval1, interval2, value):
        a = interval1[0]
        b = interval1[1]
        c = interval2[0]
        d = interval2[1]

        return c + ((d - c) / (b - a)) * (value - a)

    def set_tilt(self, tilt_angle):
        if tilt_angle < self.q3_range[0]:
            self.tilt_angle = self.q3_range[0]
        if tilt_angle > self.q3_range[1]:
            self.tilt_angle = self.q3_range[1]
# 
        # print("Calculated tilt angle (deg): {0}".format(self.tilt_angle))
        self.tilt_angle = self.tilt_angle
        # print("Calculated tilt angle (rad): {0}".format(self.tilt_angle))

    ## Camera data should be converted to cartesian coordinates (x, y, z).
    ## Based on the limitations of the arm movements, set a limited domain and set
    ## coordinates outside of this domain to the nearest possible point.
    def adjust_coordinates(self, x, y, z):
        ## Get the distance of the point
        dist = math.sqrt(y**2 + z**2)

        point = [y, z]
        # z_or = z
        ## Quantize z to the proper interval. The camera has ranges 32-86
        # z = self.quantize([32, 80], self.z_range, z)
        # print("initial z:{0}, quantized z:{1}".format(z_or, z))

        # if dist <= self.min_dist:
        #     dist = self.min_dist
        #     z = self.min_dist
        #     ## Convert the point into a unit vector
        #     point_vec = [y / dist, z / dist]
        #     ## Get the point max_dist away from the center in the direction of the vector
        #     point = list(map(lambda p: p * self.min_dist,point_vec))
        # elif dist > self.max_dist:
        #     ## Convert the point into a unit vector
        if dist > self.max_dist:
            point_vec = [y / dist, z / dist]
            ## Get the point max_dist away from the center in the direction of the vector
            point = list(map(lambda p: p * self.max_dist,point_vec))
        
        return point

    def rads3(self, angles):
        deg1 = angles[0] * (180 / math.pi)
        deg2 = angles[1] * (180 / math.pi)
        deg3 = angles[2] * (180 / math.pi)
        return [deg1, deg2, deg3]

    ## Main loop of execution
    def run(self):
        rate = rospy.Rate(10)

        if not self.simulation:
            while not rospy.is_shutdown():

                if self.arm_mode == True:
                    print("{0} -> {1}".format(self.true_z, self.target_z))
                    self.two_RIK(self.target_x, self.target_y, self.target_z)
                    self.set_tilt(self.tilt_angle)
                    ## Clamp the values
                    self.q0 = self.clamp(self.q0_range, self.q0)
                    self.q1 = self.clamp(self.q1_range, self.q1)
                    self.q2 = self.clamp(self.q2_range, self.q2)
                    self.q3 = self.clamp(self.q3_range, self.tilt_angle)
                    ## Raise the arm
                    
                    try:
                        self.move_group_arm.go((self.q0, self.q1, self.q2, self.q3), wait=False)#self.q1, self.q2, self.tilt_angle), wait=False)
                    except:
                        pass
                    rate.sleep()
                    self.move_group_gripper.go((self.gripper_target, self.gripper_target), wait=False)
                rate.sleep()
                
        # print("Current joint angles: {0}".format(self.rads3([self.q1_current, self.q2_current, self.q3_current])))
        else:
            print(self.max_dist)

            t = [0.0, 0.01, 0.02, 0.03]
            for i in t:
                print(self.quantize([0.0, 0.038], self.grip_range, i))

            y = -(self.l1*math.cos(67.0) - self.l2*math.cos(54.0))
            z = -(self.l1*math.sin(67.0) + self.l2*math.sin(54.0))
            print("Forward Kinematics: y={0}, z={1}".format(y, z))

            self.two_RIK(0, 0, z)
            # print("q1:{0}, q2:{1}\n".format(self.q1, self.q2))
            self.move_group_arm.go((self.q0, self.q1, self.q2, -100 * (math.pi/180)), wait=False)
            # self.move_group_arm.stop()
            rospy.sleep(0.5)
            ## z**2 + y**2 = dist**2
            self.two_RIK(0, 0, self.max_dist)
            # print("q1:{0}, q2:{1}\n".format(self.q1, self.q2))
            self.move_group_arm.go((self.q0, self.q1, self.q2, -100 * (math.pi/180)), wait=False)
            # self.move_group_arm.stop()
            rospy.sleep(0.5)
            self.two_RIK(0, 15.0, 18.0)
            # print("q1:{0}, q2:{1}\n".format(self.q1, self.q2))
            self.move_group_arm.go((self.q0, self.q1, self.q2, -100 * (math.pi/180)), wait=False)
            # self.move_group_arm.stop()
            rospy.sleep(5)

            ## Move in an arc along the maximum distance
            z_values = list(np.linspace(self.z_min, self.max_dist, 100))
            y_values = list(map(lambda z: math.sqrt(self.max_dist**2 - z**2), z_values))
            
            for i in range(len(z_values)):
                z = z_values[i]
                y = y_values[i]

                self.two_RIK(0, y, z)
                # print("q1:{0}, q2:{1}\n".format(self.q1, self.q2))
                self.move_group_arm.go((self.q0, self.q1, self.q2, -100 * (math.pi/180)), wait=False)
                # self.move_group_arm.stop()
                rospy.sleep(0.1)

            ## Move down
            for i in reversed(range(len(z_values))):
                z = z_values[i]
                y = 0.0

                self.two_RIK(0, y, z)
                # print("q1:{0}, q2:{1}\n".format(self.q1, self.q2))
                self.move_group_arm.go((self.q0, self.q1, self.q2, -100 * (math.pi/180)), wait=False)
                # self.move_group_arm.stop()
                rospy.sleep(0.1)

            ## Move right
            for i in range(len(z_values)):
                y = z_values[i]
                z = 0.0

                self.two_RIK(0, y, z)
                # print("q1:{0}, q2:{1}\n".format(self.q1, self.q2))
                self.move_group_arm.go((self.q0, self.q1, self.q2, -100 * (math.pi/180)), wait=False)
                # self.move_group_arm.stop()
                rospy.sleep(0.1)



if __name__ == "__main__":
    rospy.init_node("robot_action")
    handler = ActionHandler()
    handler.run()
