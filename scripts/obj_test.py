#!/usr/bin/env python

import rospy
import cv2
import torch
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import os

# Load YOLOv5 model
# model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
model_path = "../yolov5/runs/train/yolo_custom25/weights/best.pt"
model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path, force_reload=True)

def run_test():
    frame = cv2.imread("../imagedata/images_multiple/floor1.png")

    # Perform inference
    results = model(frame)

    # Extract bounding boxes
    boxes = results.xyxy[0].cpu().numpy()  # xyxy format: x1, y1, x2, y2, confidence, class
    print("box", boxes)
    names = results.names

    # Process detections (e.g., draw bounding boxes, publish results)
    for box in boxes:
        x1, y1, x2, y2, conf, cls_id = map(int, box)
        class_name = list(names[cls_id].keys())[0]
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(frame, class_name, (x1, y1 -10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

    # Display the frame with bounding boxes
    cv2.imshow("YOLOv5 Object Detection", frame)
    cv2.waitKey(0)


def main():
    run_test()

if __name__ == '__main__':
    main()
