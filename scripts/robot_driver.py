#!/usr/bin/env python3

import rospy, moveit_commander
import os
import cv2
import cv_bridge

import numpy as np
##deal with image and distance
from sensor_msgs.msg import Image, LaserScan
## deal with move
from geometry_msgs.msg import Twist, Vector3
## deal with flag
from std_msgs.msg import String

from enum import Enum, auto
from collections import deque
from itertools import *
from object_detector import Detecter

import math
import sys

########
# main idea:
# start from Idel, wait for input command, which will drive the imge to find that object and move to
# then, when close to obj, we do hand_pickup
# then, when we successfully picked up, we do return can we check if that target obj's position is higher??
# then, we turn around to find the target AR ta
# then, we close to tag, we drop off, switch back to Idel .... 

#we will have 5 action moves:
#idel
#move_to_obj
#move_to_tag
#hand_pickup
#drop_down

class next_Move(Enum):
    idel = auto()
    move_to_obj = auto()
    move_to_tag = auto()
    hand_pickup = auto()
    drop_off = auto()


class Actions(object):

    def __init__(self):

        rospy.init_node('do_actions')

        #set up for move and arm
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.cmd = Twist()
        
        # in this filel the arm control is only about drop off and stop
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # for openCV
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.bridge = cv_bridge.CvBridge()
        self.latest_image = None
        self.new_image_flag = False
        # QR-code
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

        #set up for lidar
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        #init next move
        self.move = next_Move.idel

        self.target_obj = None
        #object pool
        self.objects = {"cube", "bottle", "pen", "ball"}
        self.tag = 1

        self.state_publisher = rospy.Publisher('robot_state', String, queue_size=10)
        
        self.min_dist = None

        #PID
        self.k_linear = 0.9
        self.k_angular = 0.02

    def rest_arm(self):
        arm_joint_goal = [0.0, 0.30, 0.3, -0.9]
        gripper_joint_goal = [0.019, 0.019]

        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_arm.go(arm_joint_goal, wait=True)

        self.move_group_arm.stop()

    #set up image openCV
    def image_callback(self, data):
        self.detecter = Detecter()
        ####################################################
        # here goes the object-recognition
        self.img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.img = cv2.resize(self.img, (638, 480))
        ####################################################
        # here goes the object-recognition
        out_boxes, out_class = self.detecter.detect_image(self.img)
        # print("out class", out_class)

        self.frame_with_boxes = self.draw_boxes(self.img, out_boxes)
        self.latest_image = self.frame_with_boxes
        self.new_image_flag = True

    def main_drive(self):
        self.listenCommands()
        #using the code from line follower for to_object
        if self.move == next_Move.move_to_obj:
            self.to_object() 
        # when switched to pick_up
        if self.move == next_Move.hand_pickup:
            # first pick up
            self.pick_up()
        # when it's time for looking for AR tags
        if self.move == next_Move.move_to_tag:
            gray_img = cv2.cvtColor(self.img, cv2.COLOR_BGR2GRAY)
            self.to_tag(gray_img)
        # when ready to drop off
        if self.move == next_Move.drop_off:
            self.drop_off()
            rospy.sleep(3)
            #then rest arm and turn around
            self.rest_arm()
            self.turn_around()
            self.move = next_Move.idel

    def draw_boxes(self, image, boxes):
        for box in boxes:
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        return image

    def listenCommands(self):
        command = input("Enter command: ").lower()

        for obj in self.objects:
            if obj in command:
                if self.move == next_Move.idel:
                    self.target_obj = obj
                    self.move = next_Move.move_to_obj
                    return
                else:
                    print("Current state does not allow for user input.")
                    return
            
        print("No valid object found in the command, please type again")

    # contains find object and move to object
    def to_object(self):
        print("start moving")
        # mask = self.get_color_mask(self.object, hsv)
        # h,w,d = self.img.shape
        # M = cv2.moments(mask)

        # # if the target object is in the view, go
        # if M['m00'] > 0:
        #     cx = int(M['m10']/M['m00'])
        #     cy = int(M['m01']/M['m00'])
        
        #     min_dist = self.min_dist if self.min_dist is not None else 0.3

        #     self.cmd.linear.x = self.k_linear * min_dist
        #     error = cx - w//2
        #     self.cmd.angular.z = -self.k_angular * error
        #     self.cmd_pub.publish(self.cmd)

        # if the target object is out of the view, turn to find
        # else: 
        #     print("####  finding... ####")
        #     self.trun_to_find()

    def pick_up(self):
        self.state_publisher.publish("Start")
        #TODO: arm_control subscribe this to allow the arm control to perform

        #TODO: while hand performing, need to check the targte_obj's position
        # if in camera, target_obj is not in view:
        # self.turn_around()
        # self.move = next_Move.move_to_tag



    def drop_off(self):
        arm_joint_goal = [0, 0.65, -0.3, -0.6]
        gripper_joint_goal = [0.019, 0.019]

        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
        rospy.sleep(1.5)
        self.move_group_gripper.go(gripper_joint_goal, wait=True)


    # contains find tag and move to tag 
    def to_tag(self, gray_img):
        #process AR recognize
        (corners, ids, rejected_points) = cv2.aruco.detectMarkers(gray_img, self.aruco_dict)

        h,w = gray_img.shape[:2]
       
        # we need to find this one and move to 
        target_idx = None

        #check if it find the right tag
        if ids is not None:
            ids = np.ndarray.flatten(ids)
            
            for index, id in enumerate(ids):
                if id == self.tag:
                    target_idx = index

            if target_idx is not None:
                # start moving towards
                marker = corners[target_idx]
                cx = int(sum([corner[0] for corner in marker[0]]) /4)
                cy = int(sum([corner[1] for corner in marker[0]]) /4)

                self.cmd.linear.x = self.k_linear * self.min_dist
                error = cx - w//2
                self.cmd.angular.z = -self.k_angular * error
                self.cmd_pub.publish(self.cmd)
            
            else:
                self.turn_to_find()
        else:
            self.turn_to_find()
       
        
       

    ######
    #distance from scan data related 
    def scan_callback(self, data):

        #TODO: define our main distance  

        is_valid = lambda n: not (math.isinf(n) or math.isclose(n, 0.0))


        min_dists = list(filter(is_valid, islice(data.ranges, 563, 583)))
        # min_dists = list(islice(data.ranges, 570, 576))
        # last_9   = list(islice(reversed(data.ranges), 9))
        # min_dists = list(filter(lambda n: not (math.isinf(n) or n == 0), last_9 + first_10))

        # print(data.ranges[0])
        if min_dists == []:
            self.min_dist = None
        else:
            self.min_dist = min(min_dists)
        # self.min_dist = min(min(data.ranges[0:90]), min(data.ranges[270:]))
        
        #if too far, set to 3
        # if self.min_dist == 0:
        #     self.min_dist = 3.0

        # self.recent_dists.append(self.min_dist)

    #use a mean value from most recent 5 min_dist to find out the distance
    def check_distance(self):
        if self.min_dist:
            # ave = np.mean(self.recent_dists)

            print(self.min_dist)
            #TODO: need to figure out the stop rules

            #when it is close to obj, do switch to pick_up Move
            if self.move == next_Move.move_to_obj and self.min_dist <= 0.25:
                self.stop_moving()
                self.move = next_Move.hand_pickup

            #when it is close to tag, do switch to drop_off Move
            if self.move == next_Move.move_to_tag and self.min_dist <= 0.5:
                self.stop_moving()
                self.move = next_Move.drop_off
            


    ######
    #cmd helper funcrions, stop at the target and rotate back when done an action
    def stop_moving(self):
        self.cmd.angular.z = 0.0
        self.cmd.linear.x = 0.0
        self.cmd_pub.publish(self.cmd)

    def turn_around(self):
        #back up first
        self.cmd.linear.x = -0.1
        self.cmd.angular.z = 0.0
        self.cmd_pub.publish(self.cmd)
        rospy.sleep(3)

        #then rotate
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = np.pi / 2
        self.cmd_pub.publish(self.cmd)
        rospy.sleep(2)

         #then move forwards a bit
        self.cmd.linear.x = 0.1
        self.cmd.angular.z = 0.0
        self.cmd_pub.publish(self.cmd)
        rospy.sleep(2.5)

    def turn_to_find(self):
        self.cmd.angular.z = 0.3
        self.cmd.linear.x = 0.0
        self.cmd_pub.publish(self.cmd)



    def run(self):
        rate = rospy.Rate(10)

        # while not rospy.is_shutdown():
        #     pass
        self.rest_arm()
        rospy.sleep(3)

        # while not rospy.is_shutdown(): pass
        while not rospy.is_shutdown():

            if self.new_image_flag:
                cv2.imshow("window", self.latest_image)
                cv2.waitKey(3)
                self.new_image_flag = False
            self.main_drive()
            #here we check distanceto determin pick up or drop off
            self.check_distance()
            
            print(self.move)
            #here we check image for finding objects and tags and pickup, drop off
           
            rate.sleep()   

if __name__ == '__main__':
    node = Actions()
    node.run()
        
