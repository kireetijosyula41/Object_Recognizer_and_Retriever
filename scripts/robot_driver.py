#!/usr/bin/env python3
import sys
import rospy, moveit_commander
import os
import cv2

import numpy as np
##deal with image and distance
from sensor_msgs.msg import Image, LaserScan
## deal with move
from geometry_msgs.msg import Twist, Vector3
## deal with flag
from std_msgs.msg import String
## deal with grab
from Object_Recognizer_and_Retriever.msg import HandPos

from enum import Enum, auto
from collections import deque
from itertools import *
from obj_dect import ObjectDetection

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

        # setup the model first 
        self.obj_dect = ObjectDetection()

        #node
        rospy.init_node('do_actions')

        #set up for move 
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.cmd = Twist()
        
        #set up for arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")
        self.grip_range = [-0.010, 0.019]

        # for openCV
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.latest_image = None
        self.width = None
        self.target_center_x = None
        self.target_center_y = None
        
        
        # QR-code
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.tag = 1

        #set up for lidar
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)
        self.min_dist = None
        

        #setup for state control
        ## signal to start arm_control
        self.state_publisher = rospy.Publisher('robot_state', String, queue_size=10)
        ## signal to determine complete grabbing
        self.hand_info = rospy.Subscriber( "hand_control_topic", HandPos, self.get_hand_pos)
        self.move = next_Move.idel

        #object pool
        self.objects = {"cube", "bottle", "pen", "ball"}
        self.target_obj = ""
        

        #PID
        self.k_linear = 0.3
        self.k_angular = 0.02


    #set up image openCV
    def image_callback(self, data):
        try:
            boxes, names, frame, self.width = self.obj_dect.detect_objects(data)
            self.latest_image = frame
           
            # display the target bounding box and name
            for box in boxes:
                x1, y1, x2, y2, conf, cls_id = map(int, box)
                class_name = names[cls_id]

                if(self.target_obj in  class_name):

                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    self.target_center_x = (x1+x2)/2
                    self.target_center_y = (y1+y2)/2
                    cv2.putText(frame, class_name, (x1, y1 -10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                    break             
            
            cv2.imshow("YOLOv5 Object Detection", frame)
            cv2.waitKey(1)

        except Exception as e:
            rospy.logerr("Error processing image: %s", e)

        
    
    def main_drive(self):
        
        if self.move == next_Move.idel:
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
            gray_img = cv2.cvtColor(self.latest_image, cv2.COLOR_BGR2GRAY)
            self.to_tag(gray_img)
            self.target_center_x = None
            self.target_center_y = None
        # when ready to drop off
        if self.move == next_Move.drop_off:
            self.drop_off()
            rospy.sleep(3)
            #then rest arm and turn around
            self.rest_arm()
            self.turn_around()
            self.move = next_Move.idel
            self.stop_moving()

    
    def listenCommands(self):
        command = input("Enter command: ").lower()

        for obj in self.objects:
            if obj in command:
               
                self.target_obj = obj
                self.move = next_Move.move_to_obj
            
        print("No valid object found in the command, please type again")


    # contains find object and move to object
    def to_object(self):
        
        min_dist = self.min_dist if self.min_dist is not None else 0.3
        print("start moving:" + str(self.target_center_x))

        if self.target_center_x is not None:
            self.cmd.linear.x = self.k_linear * min_dist
            error = self.target_center_x - self.width//2
            self.cmd.angular.z = -self.k_angular * error

            self.cmd_pub.publish(self.cmd)
        # if the target object is out of the view, turn to find
        else: 
            print("Finding object")
            self.turn_to_find()


    def check_grab(self):
        consecutive_low_values = 0
        while True:

            print("GRIPGRIP:{0}".format(self.gripper_value))
            if self.gripper_value < 0.01:
                consecutive_low_values += 1
                if consecutive_low_values >= 10:
                    return True
            else:
                consecutive_low_values = 0
            
            # If the gripper value is ever above the threshold, return False
            if consecutive_low_values == 0:
                return False
            
            rospy.sleep(1)  # Sleep for 1 second before checking again


    def pick_up(self):
        self.state_publisher.publish("Start")

        if  self.check_grab():
            self.state_publisher.publish("End") 
            self.turn_around()
            self.move = next_Move.move_to_tag

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
       
        target_idx = None
        #check if it find the right tag
        if ids is not None:
            ids = np.ndarray.flatten(ids)     
            
            for index, id in enumerate(ids):
                if id == self.tag:# here we use the #1 tag
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

        is_valid = lambda n: not (math.isinf(n) or math.isclose(n, 0.0))
        min_dists = list(filter(is_valid, islice(data.ranges, 563, 583)))
    
        if min_dists == []:
            self.min_dist = None
        else:
            self.min_dist = min(min_dists)

    #use a mean value from most recent 5 min_dist to find out the distance
    def check_distance(self):
        if self.min_dist:
        
            #when it is close to obj, do switch to pick_up Move
            if self.move == next_Move.move_to_obj and self.min_dist <= 0.69:
                self.stop_moving()
                self.move = next_Move.hand_pickup

            #when it is close to tag, do switch to drop_off Move
            if self.move == next_Move.move_to_tag and self.min_dist <= 0.5:
                self.stop_moving()
                self.move = next_Move.drop_off

    ################################################################################################################
    # Helper Functions
    # ##############################################################################################################    


    ######
    # arm helper funcrionset the arm to given position
    def rest_arm(self):
        arm_joint_goal = [0.0, 0.30, 0.3, -0.9]
        gripper_joint_goal = [0.019, 0.019]

        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        self.move_group_arm.go(arm_joint_goal, wait=True)
        self.move_group_arm.stop()
    
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
        rospy.sleep(1)

    def turn_to_find(self):
        self.cmd.angular.z = 0.3
        self.cmd.linear.x = 0.0
        self.cmd_pub.publish(self.cmd)

    ####
    # some helper functions for grab
    ## Maps a value in interval1 to a value in interval2
    def quantize(self, interval1, interval2, value):
        a = interval1[0]
        b = interval1[1]
        c = interval2[0]
        d = interval2[1]

        return c + ((d - c) / (b - a)) * (value - a)

    def get_hand_pos(self, hand):
        
        self.gripper_value = max(-0.01, self.quantize([0.03, 0.09], self.grip_range, hand.gripper_value * 2))
       




    def run(self):
        rate = rospy.Rate(10)
        self.rest_arm()
        rospy.sleep(3)

        # while not rospy.is_shutdown(): pass
        while not rospy.is_shutdown():

            self.main_drive()

            #here we check distance to determin pick up or drop off
            self.check_distance()
            
            print(self.move)
           
            rate.sleep()   

if __name__ == '__main__':
    node = Actions()
    node.run()