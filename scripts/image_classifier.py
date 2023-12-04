#!/usr/bin/env python3

import os
import random
import math
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
import torch.optim as optimal
import torch.nn.functional as Func
from torchvision import models

# Helper fucntions for loading the data for use
images_path = Path('./road_signs/images')
anno_path = Path('./road_signs/annotations')

def get_images(root, file_type):
    for dir_path, dir_names, files in os.walk(root):
        for f in files: 
            if f.endswith(file_type): 
                return os.path.join(dir_path, f)

def get_image_annotations(anno_path):
    annotate = get_images(anno_path, '.xml')
    annotatelist = []
    for anno_path in annotate:
        root = ET.parse(anno_path).getroot()
        annotate_dict = {}
        annotate_dict['filename'] = Path(str(images_path) + '/'+ root.find("./filename").text)
        annotate_dict['width'] = root.find("./size/width").text
        annotate_dict['height'] = root.find("./size/height").text
        annotate_dict['class'] = root.find("./object/name").text
        annotate_dict['xmin'] = int(root.find("./object/bndbox/xmin").text)
        annotate_dict['ymin'] = int(root.find("./object/bndbox/ymin").text)
        annotate_dict['xmax'] = int(root.find("./object/bndbox/xmax").text)
        annotate_dict['ymax'] = int(root.find("./object/bndbox/ymax").text)
        annotatelist.append(annotate_dict)
    return pd.DataFrame(annotatelist)

