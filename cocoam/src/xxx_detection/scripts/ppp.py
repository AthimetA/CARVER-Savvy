#!/usr/bin/env python3

import cv2
import numpy as np

def onClick(event,x,y,flags,param):
    """Called whenever user left clicks"""
    global Running
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f'I saw you click at {x},{y}')
        Running = False

# Create window
wname = "Funky Image"
cv2.namedWindow(winname=wname)
cv2.setMouseCallback(wname, onClick)

# Load an image
img = cv2.imread(r'/home/aminballoon/Bally_ws/src/april_tag_spawner/models/Apriltag36_11_00000/materials/textures/tag36_11_00000.png')

Running = True
print(img.shape)
while Running:
    cv2.imshow(wname,img)
    cv2.waitKey(0)

cv2.destroyAllWindows()

(1024, 1024, 3)

0.45 เมตร = 1024
x เมตร = 818

def pixel_to_worldframe(pixel):
    return  pixel * 0.45 / 1024

103,103
921,103
103,921
921,921