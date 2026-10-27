#!/usr/bin/env python

import cv2
import numpy as np

# ========================= Student's code starts here =========================

# Params for camera calibration
theta = 0.0
beta = 0.0
tx = 0.0
ty = 0.0

# Function that converts image coord to world coord
def IMG2W(col, row):
    pass

# ========================= Student's code ends here ===========================

def blob_search(image_raw, color):

    # Setup SimpleBlobDetector parameters.
    params = cv2.SimpleBlobDetector_Params()

    # ========================= Student's code starts here =========================

    # Filter by Color
    params.filterByColor = False
    #params.blobColor =  

    # Filter by Area.
    params.filterByArea = True
    params.minArea = 10
    params.maxArea = 60000


    # Filter by Circularity
    params.filterByCircularity = False
    params.minCircularity = 0.01
    params.maxCircularity = 0.5

    # Filter by Inerita
    params.filterByInertia = False

    # Filter by Convexity
    params.filterByConvexity = False

    # ========================= Student's code ends here ===========================

    # Create a detector with the parameters
    detector = cv2.SimpleBlobDetector_create(params)

    # Convert the image into the HSV color space
    hsv_image = cv2.cvtColor(image_raw, cv2.COLOR_BGR2HSV)

    # ========================= Student's code starts here =========================
    if color == 'blue':
        lower = (110,50,50)     # blue lower
        upper = (130,255,255)   # blue upper
    elif color == 'green':
        lower = (45,50,90)     # green lower
        upper = (75,255,255)   # green upper
    elif color == 'orange':
        lower = (0,100,150)     # orange lower
        upper = (15,255,255)   # orange upper
    else:
        lower = (0,100,150)     # orange lower
        upper = (15,255,255)   # orange upper
  

    # Define a mask using the lower and upper bounds of the target color
    mask_image = cv2.inRange(hsv_image, lower, upper)
    #mask_imageb = cv2.inRange(hsv_image, lowerb, upperb)
    #mask_imageg = cv2.inRange(hsv_image, lowerg, upperg)
    #mask_imageo = cv2.inRange(hsv_image, lowero, uppero)

    # ========================= Student's code ends here ===========================
    
    keypoints = detector.detect(mask_image)
    #keypoints = detector.detect(mask_imageb)
    #keypoints = detector.detect(mask_imageg)
    #keypoints = detector.detect(mask_imageo)

    # Find blob centers in the image coordinates
    blob_image_center = []
    num_blobs = len(keypoints)
    print(num_blobs)
    for i in range(num_blobs):
        blob_image_center.append((keypoints[i].pt[0],keypoints[i].pt[1]))

    # ========================= Student's code starts here =========================

    # Draw the keypoints on the detected block
    im_with_keypoints = cv2.drawKeypoints(image_raw, keypoints, None)

    # ========================= Student's code ends here ===========================

    xw_yw = []

    if(num_blobs == 0):
        print("No block found!")
    else:
        # Convert image coordinates to global world coordinate using IM2W() function
        for i in range(num_blobs):
            xw_yw.append(IMG2W(blob_image_center[i][0], blob_image_center[i][1]))


    cv2.namedWindow("Camera View")
    cv2.imshow("Camera View", image_raw)
    cv2.namedWindow(f"Mask View - {color}")
    cv2.imshow(f"Mask View - {color}", mask_image)
    # cv2.imshow("Mask View", mask_imageb)
    #cv2.imshow("Mask View", mask_imageg)
    # cv2.imshow("Mask View", mask_imageo)
    cv2.namedWindow(f"Keypoint View - {color}")
    cv2.imshow(f"Keypoint View - {color}", im_with_keypoints)

    cv2.waitKey(2)

    return xw_yw
