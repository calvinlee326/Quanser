'''hardware_test_intelrealsense.py

This example demonstrates how to read and display depth & RGB image data
from the Intel Realsense camera.
'''
import time
import cv2
from pal.products.qcar import QCarRealSense, IS_PHYSICAL_QCAR

if not IS_PHYSICAL_QCAR:
    import qlabs_setup
    qlabs_setup.setup()


#Initial Setup
runTime = 20.0 # seconds
max_distance = 2 # meters (for depth camera)

with QCarRealSense(mode='RGB&DEPTH') as myCam:
    t0 = time.time()
    while True:
    # while time.time() - t0 < runTime: 
        myCam.read_RGB()
        cv2.imshow('My RGB', myCam.imageBufferRGB)
        # myCam.read_depth(dataMode='M')
        # cv2.imshow('My Depth', myCam.imageBufferDepthPX/max_distance)
        cv2.waitKey(100)
