#!/usr/bin/env python3

import rospy, cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

import os
import random
import math
from datetime import datetime
from collections import Counter
import pandas as pd
import numpy as np

import cv2
from PIL import Image
from pathlib import Path
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from sklearn.model_selection import train_test_split
import xml.etree.ElementTree as ET

import torch
from torch.utils.data import Dataset, DataLoader
import torch.optim as optim
import torch.nn as nn
import torch.nn.functional as F
from torchvision import models

class ImageDataset(Dataset):
    def __init__(self, paths, bb, y, transforms=False):
        self.transforms = transforms
        self.paths = paths.values
        self.bb = bb.values
        self.y = y.values
        
    def normalize(self, im):
        imagenet_stats = np.array([[0.485, 0.456, 0.406], [0.229, 0.224, 0.225]])
        return (im - imagenet_stats[0])/imagenet_stats[1]

    def __len__(self):
        return len(self.paths)

    def __getitem__(self, idx):
        path = self.paths[idx]
        y_class = self.y[idx]
        x = cv2.imread(str(path)).astype(np.float32)
        x = cv2.cvtColor(x, cv2.COLOR_BGR2RGB) / 255
        # x, y_bb = transformsXY(path, self.bb[idx], self.transforms)
        x = self.normalize(x)
        x = np.rollaxis(x, 2)
        return x, y_class, self.bb[idx]


class BB_model(nn.Module):
    def __init__(self):
        super(BB_model, self).__init__()
        resnet = models.resnet34(pretrained=True)
        layers = list(resnet.children())[:8]
        self.features1 = nn.Sequential(*layers[:6])
        self.features2 = nn.Sequential(*layers[6:])
        self.classifier = nn.Sequential(nn.BatchNorm1d(512), nn.Linear(512, 5))
        self.bb = nn.Sequential(nn.BatchNorm1d(512), nn.Linear(512, 4))

    def forward(self, x):
        x = self.features1(x)
        x = self.features2(x)
        x = F.relu(x)
        x = nn.AdaptiveAvgPool2d((1, 1))(x)
        x = x.view(x.shape[0], -1)
        return self.classifier(x), self.bb(x)


class Follower:
        def __init__(self):
                # set up ROS / OpenCV bridge
                self.bridge = cv_bridge.CvBridge()
                # initalize the debugging window
                cv2.namedWindow("window", 1)
                # subscribe to the robot's RGB camera data stream
                self.image_sub = rospy.Subscriber('camera/rgb/image_raw', Image, self.image_callback)
                self.latest_image = None
                self.new_image_flag = False

        def image_callback(self, msg):
                # converts the incoming ROS message to OpenCV format and HSV (hue, saturation, value)
                image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
                # this erases all pixels that aren't yellow

                im = cv2.resize(image, (638, 480))
                cv2.imwrite('./prt_resized.png', cv2.cvtColor(im, cv2.COLOR_RGB2BGR))
                # test Dataset
                test_ds = ImageDataset(pd.DataFrame([{'path':'./prt_resized.png'}])['path'],pd.DataFrame([{'bb':np.array([0,0,0,0])}])['bb'],pd.DataFrame([{'y':[0]}])['y'])
                x, y_class, y_bb = test_ds[0]
                xx = torch.FloatTensor(x[None,])
                # prediction
                out_class, out_bb = self.model(xx)
                print("out class", self.rev_class_dict[torch.argmax(out_class, 1).tolist()[0]], out_class)
                print("bounding box", out_bb)

        def run(self):
                rate = rospy.Rate(30)  # Set an appropriate rate (e.g., 30Hz)
                while not rospy.is_shutdown():
                        if self.new_image_flag:
                                cv2.imshow("window", self.latest_image)
                                cv2.waitKey(3)
                                self.new_image_flag = False
                        rate.sleep()
        
if __name__ == '__main__':

        rospy.init_node('line_follower')
        follower = Follower()
        follower.run()
