#!/usr/bin/env python3

import rospy
import cv2
import cv_bridge

from sensor_msgs.msg import Image

from object_detector import Detecter


class Test(object):

    def __init__(self):

        rospy.init_node('test1')
        self.detecter = Detecter()

        # for openCV
        self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
        self.bridge = cv_bridge.CvBridge()
     


    #set up image openCV
    def image_callback(self, data):
        self.img = self.bridge.imgmsg_to_cv2(data, desired_encoding='bgr8')
        ####################################################
        # here goes the object-recognition
        out_boxes, out_class = self.detecter.detect_image(self.img)
        print("out class", out_class)

        self.frame_with_boxes = self.draw_boxes(self.img, out_boxes)

    def draw_boxes(self, image, boxes):
        for box in boxes:
            x1, y1, x2, y2 = map(int, box)
            cv2.rectangle(image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        return image
        

    def run(self):
        rate = rospy.Rate(10)
    
        while not rospy.is_shutdown():
            
            rospy.spin()
            cv2.imshow("Camera Feed with Object Detection", self.frame_with_boxes)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            rate.sleep()   

if __name__ == '__main__':
    node = Test()
    node.run()
        
