#!/usr/bin/env python

import rospy
import cv2
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Load YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

def image_callback(msg):
    try:
        # Convert ROS Image message to OpenCV image
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(msg, "bgr8")

        # Perform inference
        results = model(frame)

        # Extract bounding boxes
        boxes = results.xyxy[0].numpy()  # xyxy format: x1, y1, x2, y2, confidence, class

        # Process detections (e.g., draw bounding boxes, publish results)
        for box in boxes:
            x1, y1, x2, y2, conf, cls = map(int, box)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # Display the frame with bounding boxes
        cv2.imshow("YOLOv5 Object Detection", frame)
        cv2.waitKey(1)

    except Exception as e:
        rospy.logerr("Error processing image: %s", e)

def main():
    rospy.init_node('yolov5_object_detection')
    rospy.Subscriber("/camera/rgb/image_raw", Image, image_callback)
    rospy.spin()

if __name__ == '__main__':
    main()
