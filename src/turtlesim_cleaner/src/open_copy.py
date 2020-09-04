#!/usr/bin/env python

import numpy as np
import cv2

image_name = "tree"

print 'read an image from file'
img = cv2.imread("../images/"+image_name+".jpg")

print 'create a window holder for the image'
cv2.namedWindow("Image", cv2.WINDOW_NORMAL)

print 'display the image'
cv2.imshow("Image", img)

print 'pres a key inside to take a copy'
cv2.waitKey(0)

print 'image copied to folder images/copy/'
cv2.imwrite("../images/copy/"+image_name+"-copy.jpg", img)