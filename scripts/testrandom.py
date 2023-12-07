#!/usr/bin/env python3

import numpy as np
import cv2

test_image = np.random.randint(0,225, (480,640,3), dtype=np.uint8)
cv2.imshow("Test Window", test_image)
cv2.waitKey(0)
cv2.destroyAllWindows()