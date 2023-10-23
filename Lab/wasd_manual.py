# import lib
import pygame
from pal.products.qcar import QCar
from pal.utilities.math import *
import numpy as np
import time
from mini_cameras_view import camPreview
import threading
import sys
# create qcar
myCar = QCar(readMode=0)

def wasd_drive():

    # init Pygame
    pygame.init()

    # setup screen size
    screen_width = 800
    screen_height = 600
    screen = pygame.display.set_mode((screen_width, screen_height))
    pygame.display.set_caption("QCar Manual Control")

    # set up +- increment and maximum
    throttle_increment = 0.0001
    steering_increment = 0.005
    max_throttle = 0.2
    min_throttle = -0.2
    max_steering = 0.5
    min_steering = -0.5

    # default speed and direction
    throttle = 0.0
    steering = 0.02

    def stop():
        throttle = 0.0
        steering = 0.02

    # main program
    runTime = 10.0 # seconds
    t0 = time.time()
    
    # Default LED status
    LEDs = np.array([0, 0, 0, 0, 0, 0, 0, 0])
    isReverse = False
    headlights_on = False
    s_key_pressed = False

    turn_signal_timer = 0.0
    turn_signal_interval = 0.5
    turn_signal_state = 0 
    while True:
    # while time.time() - t0  < runTime :
        # t = time.time()
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYUP:
                if event.key == pygame.K_a or  event.key == pygame.K_d:
                    steering = 0.0

        keys = pygame.key.get_pressed()

        # steering
        if keys[pygame.K_w]:
            if not isReverse:
                throttle += throttle_increment
                throttle = min(throttle, max_throttle)
                LEDs[4] = 0
            if isReverse:
                throttle -= throttle_increment
                throttle = min(throttle, max_throttle)

        elif keys[pygame.K_s]:
            if throttle > 0 and not isReverse:
                throttle -= throttle_increment
                throttle = max(throttle, min_throttle)
                LEDs[4] = 1
                s_key_pressed = True  # Set flag when 'S' key is pressed
            elif throttle < 0 and isReverse:
                throttle += throttle_increment
                throttle = max(throttle, min_throttle)
            else:
                s_key_pressed = False  # Reset flag when 'S' key is released

        if not keys[pygame.K_s] and not s_key_pressed:
            LEDs[4] = 0

        if keys[pygame.K_r]:
            isReverse = True
            throttle = 0
            LEDs[4] = 0  # Turn off LED[4] when 'R' key is pressed
            LEDs[5] = 1
        elif keys[pygame.K_f]:
            isReverse = False
            throttle = 0
            LEDs[5] = 0
        elif keys[pygame.K_f]:
            isReverse = False
            throttle = 0
            LEDs[5] = 0

        if keys[pygame.K_d]:
            steering -= steering_increment
            steering = max(steering, min_steering)
        elif keys[pygame.K_a]:
            steering += steering_increment
            steering = min(steering, max_steering)
        
        if keys[pygame.K_q]:
            steering , throttle = 0.0 , 0.0
            LEDs[5] = 0
        
        if keys[pygame.K_h]:
            headlights_on= not headlights_on
            LEDs[6] = 1 if headlights_on else 0
            LEDs[7] = 1 if headlights_on else 0
            
        # based on direction and speed update led control
        if steering > 0.15:
            if turn_signal_timer >= turn_signal_interval:
                turn_signal_state = 1 - turn_signal_state
                turn_signal_timer = 0.0
            if turn_signal_state:
                LEDs[0] = 1
                LEDs[2] = 1
            else:
                LEDs[0] = 0
                LEDs[2] = 0
        elif steering < -0.15:
            if turn_signal_timer >= turn_signal_interval:
                turn_signal_state = 1 - turn_signal_state
                turn_signal_timer = 0.0
            if turn_signal_state:
                LEDs[1] = 1
                LEDs[3] = 1
            else:
                LEDs[1] = 0
                LEDs[3] = 0
        else:
            LEDs[0] = 0
            LEDs[1] = 0
            LEDs[2] = 0
            LEDs[3] = 0

        if throttle < 0:
            LEDs[5] = 1
        else:
            LEDs[5] = 0
        

        # update qcar control
        myCar.read_write_std(throttle=throttle, steering=steering, LEDs = LEDs)
        print(throttle, steering, isReverse, LEDs)

# update turnled timer
    turn_signal_timer += 0.01

t1 = threading.Thread(target=wasd_drive)
t2 = threading.Thread(target=camPreview,args=[["front","back"]])

try:
    t1.start()
    t2.start()
except KeyboardInterrupt:
        myCar.terminate()
        # exitPygame
        pygame.quit()
        sys.exit()
    
        


