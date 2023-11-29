#!/usr/bin/env python3
import rospy
import numpy as np
import math
import cv2
from depthAI_utils.HandTrackerRenderer import HandTrackerRenderer
from depthAI_utils.HandTracker import HandTracker
from itertools import chain
##our msg package
from Object_Recognizer_and_Retriever.msg import HandPos
from geometry_msgs.msg import Point

####################
# this class is used for hand tracking when needed.
# theta1 will be calculated by x,y.
# theta2 ,3 will be calculated mainly by z
# theta 4 will be calculated by wrist and palm center vertical angle.

# Will publich:
# 1. the 3D position(x,y,z) for arm to behave
# 2. the tilt data for theta4
# 3. will define the grab and release
####################

class HandCapture:
    def __init__(self):
        # ROS Publisher setup
        self.pub = rospy.Publisher('hand_control_topic', HandPos, queue_size=10)
        rospy.init_node('hand_control_publisher', anonymous=True)

        self.tracker = HandTracker()
        self.renderer = HandTrackerRenderer(tracker=self.tracker)

        self.grab_mode = False # suppose we will have state control: only in grab_mode can do hand tracking
        self.initial_distance = None # this should be reset everytime


   # this will be used for gripper mimic
    def calculate_pinch_distance(self, points):
        thumb_tip = points[4]
        index_tip = points[8]
        distance = np.sqrt((thumb_tip[0] - index_tip[0]) ** 2 + (thumb_tip[1] - index_tip[1]) ** 2 + (thumb_tip[2] - index_tip[2]) ** 2)
        return distance

    def calculate_tilt(self, points):
        wrist = points[0]
        #palm = math.sum([points[0], points[2], points[5]])/3 # suppose we only do horizontal pinch
        palm = points[4]

        d = palm - wrist
        d_xy = [d[0], d[1], 0]
        # Calculate magnitudes of vectors
        mag_d = math.sqrt(d[0]**2 + d[1]**2 + d[2]**2)
        mag_d_xy = math.sqrt(d_xy[0]**2 + d_xy[1]**2)

        if mag_d == 0 or mag_d_xy == 0:
            return 0

        # Calculate dot product
        dot_product = d[0]*d_xy[0] + d[1]*d_xy[1]
        # Calculate the angle in radians
        angle_rad = math.acos(dot_product / (mag_d * mag_d_xy))

        if d[2] > 0:
            angle_rad = -angle_rad

        angle_deg = math.degrees(angle_rad)

        return  angle_deg



    def map_pinch_to_gripper(self, pinch_distance, initial_distance):
        gripper_max_open = 0.038  # Gripper range (in meters)
        # Map the pinch distance to the gripper range
        pinch_distance = min(pinch_distance, initial_distance)
        normalized_pinch = pinch_distance /  initial_distance if initial_distance != 0 else 0
        gripper_position = normalized_pinch * gripper_max_open

        # Ensure gripper position is within the valid range
        gripper_position = max(min(gripper_position, gripper_max_open), 0)

        return gripper_position

    def capture_and_process(self):
        initial_hand_position = None # to store the first catched hand data
        hand_missing_counter = 0
        HAND_MISSING_THRESHOLD = 30  # Number of frames to wait before resetting

        try:
            while not rospy.is_shutdown():
                # Check for the presence of the left hand
                frame, hands, bag = self.tracker.next_frame()

                if hands:
                    hand_missing_counter = 0  # Reset counter as hand is present

                    camera_height = 80 #80cm above the table ground
                    wrist_xyz0 = hands[0].xyz / 10.0 # in cm
                    #print(wrist_xyz0[2])
                    wrist_xyz0[2] =camera_height -wrist_xyz0[2] # this is the height from floor
                    if  wrist_xyz0[2] < 0:  wrist_xyz0[2] = 0

                    if initial_hand_position is None:
                        initial_hand_position = wrist_xyz0

                     # Calculate relative X and Y, keep Z absolute based on the first time hand been detected
                    relative_position = wrist_xyz0 - initial_hand_position
                    relative_position[2] = wrist_xyz0[2]

                    points = hands[0].get_rotated_world_landmarks()
                    points = points + relative_position

                    if self.initial_distance is None:
                        self.initial_distance = self.calculate_pinch_distance(points)

                    gripper_value = self.map_pinch_to_gripper(self.calculate_pinch_distance(points), self.initial_distance)

                    tilt_angle = self.calculate_tilt(points)

                    #print(points[0])#publish the wrist position to ROS
                    #print(gripper_value/2)#publish the gripper value to ROS
                    #print(tilt_value)#publish the tilt degree value(up is positive, down is negative) to ROS

                    # # Publish hand data
                    msg = HandPos()
                    msg.point = Point(points[0][0], points[0][1], points[0][2]) 
                    msg.gripper_value = gripper_value  # Set the gripper value
                    msg.tilt_angle = tilt_angle  # Set the tilt angle

                    self.pub.publish(msg)


                else:
                    hand_missing_counter += 1
                    if hand_missing_counter > HAND_MISSING_THRESHOLD:
                        initial_hand_position = None  # Reset initial position after hand is missing for a while
                        self.initial_distance = None


                if frame is None: break

                # Draw hands
                frame = self.renderer.draw(frame, hands, bag)
                key = self.renderer.waitKey(delay=1)
                if key == 27 or key == ord('q'):
                    break

            self.renderer.exit()
            self.tracker.exit()
        finally:
            cv2.destroyAllWindows()

if __name__ == '__main__':
    hand_capture = HandCapture()
    hand_capture.capture_and_process()




