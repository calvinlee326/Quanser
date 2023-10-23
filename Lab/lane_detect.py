'''hardware_test_csi_cameras.py

This example demonstrates how to read and display images from the CSI cameras.
'''
import time
import cv2
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
from pal.products.qcar import QCarCameras, IS_PHYSICAL_QCAR

if not IS_PHYSICAL_QCAR:
    import qlabs_setup
    qlabs_setup.setup()

# Lane detection using Hough Line Transform
def detect_lanes(frame):
    # Preprocess the frame (convert to grayscale and apply Gaussian blur)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    
    # Perform edge detection (Canny)
    edges = cv2.Canny(blurred, 50, 150)
    
    # Define ROI (Region of Interest)
    height, width = frame.shape[:2]
    roi_vertices = [(0, height), (width / 2, height / 2), (width, height)]
    mask = np.zeros_like(edges)
    cv2.fillPoly(mask, [np.array(roi_vertices, np.int32)], 255)
    masked_edges = cv2.bitwise_and(edges, mask)
    
    # Use Hough Line Transform to detect lines
    lines = cv2.HoughLinesP(masked_edges, 1, np.pi / 180, 100, minLineLength=10, maxLineGap=250)
    
    # Draw the detected lines on the frame
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(frame, (x1, y1), (x2, y2), (0, 0, 255), 5)


#Initial Setup
runTime = 100.0 # seconds

cameras = QCarCameras(
    enableBack=False,
    enableFront=True,
    enableLeft=False,
    enableRight=False,
)

with cameras:
    t0 = time.time()
    while time.time() - t0 < runTime:
        cameras.readAll()

        for i, c in enumerate(cameras.csi):
            if c is not None:

                detect_lanes(c.imageData)
                cv2.imshow(('CSI '+str(i)+':'), c.imageData)

        # Note: You can also access specific cameras directly if you know
        # which one you want. For example:
        #   cv2.imshow('CSI Front', cameras.csiFront.imageData)

        cv2.waitKey(100)
