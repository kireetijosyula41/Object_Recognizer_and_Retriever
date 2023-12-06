#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge

from sensor_msgs.msg import Image

from object_detector import Detecter


class Test(object):

    def __init__(self):

        rospy.init_node('test1')
        cv2.namedWindow("window", 1)
        self.detecter = Detecter()

        # for openCV
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.bridge = cv_bridge.CvBridge()
     
        self.latest_image = None
        self.new_image_flag = False
    #set up image openCV
    def image_callback(self, data):
        self.img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        self.image = cv2.resize(self.img, (638, 480))
        ####################################################
        # here goes the object-recognition
        out_boxes, out_class = self.detecter.detect_image(self.image)
        print("out class", out_class)

        self.frame_with_boxes = self.draw_boxes(self.image, out_boxes)
        self.latest_image = self.frame_with_boxes
        self.new_image_flag = True

    def draw_boxes(self, image, boxes):
        for box in boxes:
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        return image

    def run(self):
        rate = rospy.Rate(30)  # Set an appropriate rate (e.g., 30Hz)
        while not rospy.is_shutdown():      
            if self.new_image_flag:
                cv2.imshow("window", self.latest_image)
                cv2.waitKey(3)
                self.new_image_flag = False
            rate.sleep()   

if __name__ == '__main__':
    node = Test()
    node.run()
        
