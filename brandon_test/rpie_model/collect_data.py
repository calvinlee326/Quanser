import argparse
import io
import pygame
import threading
import time
import numpy as np
import cv2
from brandon_test.rpie_model.car_control import backward, stop, forward, left, right
from pal.products.qcar import QCar, QCarRealSense
import DetectLane as DetectLane
import sys
sys.path.append('/home/nvidia/Documents/brandon_test/rpie_model')



myCar = QCar(readMode=0)
is_capture_running = False
key = 'stop'

class SplitFrames(object):
    
    def __init__(self):
        self.frame_num = 0
        self.output = None

    def write(self, buf):
        global key
        if buf.startswith(b'\xff\xd8'):                            
            # Start of new frame; close the old one (if any) and
            # open a new output
            if self.output:
                self.output.close()
            self.frame_num += 1
            self.output = io.open('%s_image%s.jpg' % (key,time()), 'wb')           
        self.output.write(buf)

def qcar_capture():
    global is_capture_running, key

    print("Start capture")
    is_capture_running = True
    camera_realsense_rgb = QCarRealSense(mode='RGB')

    while is_capture_running:
        # Capture image from the QCar's camera
        camera_realsense_rgb.read_RGB()
        img = cv2.resize(camera_realsense_rgb.imageBufferRGB, (160, 120))
        filename = f'{key}_{int(time.time())}.jpg'
        cv2.imwrite(filename, img)

        time.sleep(0.1)

    print("Stop capture")

def qcar_control(): 
    global is_capture_running, key

    pygame.init()
    screen = pygame.display.set_mode((1, 1))
    print("Start control!")
    stop()

    while is_capture_running:
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                key_input = pygame.key.get_pressed()
                if key_input[pygame.K_w]:
                    key = 'forward'
                    forward()
                elif key_input[pygame.K_a]:
                    key = 'left'
                    left()
                elif key_input[pygame.K_d]:
                    key = 'right'
                    right()
                elif key_input[pygame.K_s]:
                    key = 'backward'
                    backward()
                elif key_input[pygame.K_k]:
                    key = 'stop'
                    stop()
            elif event.type == pygame.KEYUP:
                stop()
                key = 'stop'

if __name__ == '__main__':
    print("Start data collection")
    capture_thread = threading.Thread(target=qcar_capture)
    control_thread = threading.Thread(target=qcar_control)

    capture_thread.start()
    control_thread.start()

    capture_thread.join()
    control_thread.join()

    print("Data collection complete")
