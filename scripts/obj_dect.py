#!/usr/bin/env python3
import torch
from cv_bridge import CvBridge

class ObjectDetection:
    def __init__(self, conf_thresh=0.25):
        self.custom = False
        self.conf_thresh = conf_thresh
        # Load YOLOv5 model
        ### CUSTOM MODEL
        if self.custom:
            model_path = "./yolo/runs/train/exp4/weights/best.pt"
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

        filtered_boxes = []
        for box in boxes:
            if box[4] >= self.conf_thresh:
                filtered_boxes.append(box)
       
        return np.array(filtered_boxes), names, frame, w