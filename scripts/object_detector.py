#!/usr/bin/env python3

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
from sklearn.model_selection import train_test_splitpip 
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
        resnet = models.resnet50(pretrained=True)
        layers = list(resnet.children())[:8]
        self.features1 = nn.Sequential(*layers[:6])
        self.features2 = nn.Sequential(*layers[6:])
        self.classifier = nn.Sequential(nn.BatchNorm1d(2048), nn.Linear(2048, 5))
        self.bb = nn.Sequential(nn.BatchNorm1d(2048), nn.Linear(2048, 4))

    def forward(self, x):
        x = self.features1(x)
        x = self.features2(x)
        x = F.relu(x)
        x = nn.AdaptiveAvgPool2d((1, 1))(x)
        x = x.view(x.shape[0], -1)
        return self.classifier(x), self.bb(x)


class Detecter:
    def __init__(self):
        self.model = BB_model()
        self.model.load_state_dict(torch.load('../trained_model'))
        self.model.eval()
        self.class_dict = {'ball': 0, 'cube': 1, "bottle": 2, "pen": 3, "nothing": 4}
        self.rev_class_dict = {value: key for key, value in self.class_dict.items()}
    
    def detect_image(self, image):
        im = cv2.resize(image, (638, 480))
        cv2.imwrite('./prt_resized.png', cv2.cvtColor(im, cv2.COLOR_RGB2BGR))
        # test dataset
        test_ds = ImageDataset(pd.DataFrame([{'path':'./prt_resized.png'}])['path'],pd.DataFrame([{'bb':np.array([0,0,0,0])}])['bb'],pd.DataFrame([{'y':[0]}])['y'])
        x, y_class, y_bb = test_ds[0]
        xx = torch.FloatTensor(x[None,])
        # prediction
        out_class, out_bb = self.model(xx)
        # print("out class", self.rev_class_dict[torch.argmax(out_class, 1).tolist()[0]], out_class)
        # print("bounding box", out_bb)
        return out_bb, torch.argmax(out_class)


