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
<<<<<<< Updated upstream
        ## Minimum and maximum joint angles for the robot arm
        self.q1_range = [-162 * (math.pi / 180), 162 * (math.pi / 180)]
        self.q2_range = [-102 * (math.pi / 180), 83 * (math.pi / 180)]
        self.q3_range = [-54 * (math.pi / 180), 79 * (math.pi / 180)]
        self.tilt_range = [-90, 90]

=======
>>>>>>> Stashed changes
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

        ## Storing values for target angles
        self.q0 = 0
        self.q1 = 0
        self.q2 = 0
        self.q3 = -102.0 * (math.pi / 180)

        ## Storing values for current angles
        self.q1_current = 0
        self.q2_current = 0
        self.q3_current = 0

        ## Storing values for target position
        self.target_x = 0
        self.target_y = 5
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
        # Calculate the distance between the end effector and the origin
        distance = math.sqrt(z**2 + y**2)
        ## Adjust coordinates
        adjusted = self.adjust_coordinates(x, y, z)
        x = adjusted[0]
        y = adjusted[1]
        z = adjusted[2]
        print(adjusted)
        # dist = math.sqrt(z*z + y*y)
        print("Attempting to move to position {0} {1} {2}".format(x, y, z))

<<<<<<< Updated upstream
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
=======
        y_true = y
        z_true = z
        y = abs(y)
        z = abs(z)

        ## Calculate the angle to turn the arm
        if x == 0:
            self.q0 = 0.0
        elif x > 0:
            if y == 0:
                self.q0 = -math.pi / 2
            else:
                self.q0 = -math.atan(x / y)
        else:
            if y == 0:
                self.q0 = math.pi / 2
            else:
                self.q0 = math.atan(x / y)
>>>>>>> Stashed changes

        
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
            print("Q1")
            q1a = (math.pi / 2) - q1a
            q2a = -((math.pi / 2) - q2a)
            q1b = (math.pi / 2) - q1b
            q2b = -((math.pi / 2) - q2b)
        elif y_true == 0:
            print("Q1/Q2 (y=0)")
            q1a = q1a
            q2a = -((math.pi / 2) - q2a)
            q1b = q1b
            q2b = -((math.pi / 2) - q2b)
        ## Q2. Mirror q1
        elif y_true < 0.0 and z_true > 0.0:
            print("Q2")
            print("q1b: {0}, q2b:{1}, q3:{2}".format(np.degrees(q1b), np.degrees(q2b), self.q3* (180 / math.pi)))
            print("q1a: {0}, q2a:{1}, q3:{2}".format(np.degrees(q1a), np.degrees(q2a), self.q3* (180 / math.pi)))

            # q1a = (-q1a) - (math.pi / 2)
            # q2a = (-(math.pi / 2) + q2a)
            # q1b = (math.pi / 2) - q1b
            # q2b = (-(math.pi / 2) + q2b)
            q1a = -((math.pi / 2) - q1a)
            q2a = (-((math.pi / 2) - q2a))
            q1b = -((math.pi / 2) - q1b)
            q2b = (-((math.pi / 2) - q2b))

        self.q1 = q1a
        self.q2 = q2a

        print("q0: {3}, q1b: {0}, q2b:{1}, q3:{2}".format(np.degrees(q1b), np.degrees(q2b), self.q3* (180 / math.pi), self.q0))
        print("q0: {3}, q1a: {0}, q2a:{1}, q3:{2}".format(np.degrees(q1a), self.q2* (180 / math.pi), self.q3* (180 / math.pi), self.q0))

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
    def adjust_coordinates(self, x, y, z):
        ## Get the distance of the point
        dist = math.sqrt(x**2 + y**2 + z**2)

        point = [x, y, z]
        if dist <= self.min_dist:
            dist = self.min_dist
            z = self.min_dist
            ## Convert the point into a unit vector
            point_vec = [x / dist, y / dist, z / dist]
            ## Get the point max_dist away from the center in the direction of the vector
            point = list(map(lambda p: p * self.min_dist,point_vec))
        elif dist > self.max_dist:
            ## Convert the point into a unit vector
            point_vec = [x / dist, y / dist, z / dist]
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
        if not self.simulation:
            while not rospy.is_shutdown():
                self.two_RIK(self.target_x, self.target_y, self.target_z)
                self.set_tilt(self.tilt_angle)
                ## Raise the arm
<<<<<<< Updated upstream
                self.move_group_arm.go((0, self.q1, self.q2, self.tilt_angle), wait=False)
                self.move_group_arm.stop()
                rospy.sleep(1)
=======
                self.move_group_arm.go((0, self.q1, self.q2, -100 * (math.pi/180)), wait=False)
                rospy.sleep(0.1)
>>>>>>> Stashed changes
        # print("Current joint angles: {0}".format(self.rads3([self.q1_current, self.q2_current, self.q3_current])))
        else:
            print(self.max_dist)

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

            # ## Test coordinate adjustment
            # p = [18, 20, 0]
            # p_a = self.adjust_coordinates(p[0], p[1], p[2])
            # print('Point:{0}\nValue:{1}\n'.format(p, p_a))

            # self.two_RIK(0, 10, 25)
            # # print("q1:{0}, q2:{1}\n".format(self.q1, self.q2))
            # self.move_group_arm.go((self.q0, self.q1, self.q2, -100 * (math.pi/180)), wait=False)
            # # self.move_group_arm.stop()
            # rospy.sleep(0.25)

            # ## Move right
            # y_values = list(np.linspace(0, 16.9, 100))
            
            # for i in reversed(range(len(y_values))):
            #     z = y_values[i]
            #     y = y_values[i]

            #     self.two_RIK(0, y, z)
            #     # print("q1:{0}, q2:{1}\n".format(self.q1, self.q2))
            #     self.move_group_arm.go((self.q0, self.q1, self.q2, -100 * (math.pi/180)), wait=False)
            #     # self.move_group_arm.stop()
            #     rospy.sleep(0.8)

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
