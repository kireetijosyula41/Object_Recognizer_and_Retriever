#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge

##deal with image and distance
from sensor_msgs.msg import Image

from object_detector import Follower


class Test(object):

    def __init__(self):

        rospy.init_node('test1')

        # for openCV
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.bridge = cv_bridge.CvBridge()
     


    #set up image openCV
    def image_callback(self, data):
        self.img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        ####################################################
        # here goes the object-recognition
        Follower.detect_image(self.img)
        

    def run(self):
        rate = rospy.Rate(10)
    
        while not rospy.is_shutdown():
            
            rospy.spin()
            rate.sleep()   

if __name__ == '__main__':
    node = Test()
    node.run()
        
