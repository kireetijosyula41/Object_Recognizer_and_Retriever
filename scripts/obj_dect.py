#!/usr/bin/env python3
import torch
from cv_bridge import CvBridge

class ObjectDetection:
    def __init__(self):
        self.custom = False
        # Load YOLOv5 model
        ### CUSTOM MODEL
        if self.custom:
            model_path = "./yolo/runs/train/yolo_custom12/weights/best.pt"
            self.model = torch.hub.load('yolo/', 'custom', path=model_path, source='local')
            self.bridge = CvBridge()
        ### Pretrained model    
        else:
            self.model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)
            self.bridge = CvBridge()

    def detect_objects(self, ros_image):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        
        h,w,d = frame.shape # we need w for moving towards

        # Perform inference
        results = self.model(frame)
        names = results.names
        # Extract bounding boxes
        boxes = results.xyxy[0].cpu().numpy()  # xyxy format: x1, y1, x2, y2, confidence, class
       
        return boxes, names, frame, w