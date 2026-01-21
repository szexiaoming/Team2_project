# Copyright 1996-2023 Cyberbotics Ltd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

'''Routine to detect a fall and recover from it.'''
from controller import Robot, Motion
from .accelerometer import Accelerometer
from .motion_library import MotionLibrary
from .finite_state_machine import FiniteStateMachine
from .current_motion_manager import CurrentMotionManager


import imghdr
from controller import Robot
import sys
sys.path.append('..')
import numpy as np
import cv2
import base64
from utils.camera_bottom import CameraBottom
import math
from utils.motion_library import MotionLibrary



class BorderDetection:
    def __init__(self, time_step, robot):
        self.time_step = time_step
        self.robot = robot
        # the Finite State Machine (FSM) is a way of representing a robot's behavior as a sequence of states
        self.fsm = FiniteStateMachine(
            states=['NO_LINE', 'BLOCKING_MOTION',
                    'FRONT_FALL', 'BACK_FALL', 'SIDE_FALL','LINE_DETECTED'],
            initial_state='NO_LINE',
            actions={
                'NO_LINE': self.wait,
                'BLOCKING_MOTION': self.pending,
                'FRONT_FALL': self.front_fall,
                'BACK_FALL': self.back_fall,
                'SIDE_FALL': self.wait,
                'LINE_DETECTED': self.turn_around,}
        )
        self.accelerometer = Accelerometer(robot, self.time_step)
        # Shoulder roll motors to recover from a side fall
        self.RShoulderRoll = robot.getDevice('RShoulderRoll')
        self.LShoulderRoll = robot.getDevice('LShoulderRoll')
        self.current_motion = CurrentMotionManager()
        self.library = MotionLibrary()


        self.camera = robot.getDevice('CameraBottom')
        self.height = self.camera.getHeight()
        self.width = self.camera.getWidth()


    def get_image(self):
        """Get an openCV image (BGRA) from a Webots camera."""
        return np.frombuffer(self.camera.getImage(), np.uint8).reshape((self.height, self.width, 4))
    
    def check(self):
        '''Check if the robot has fallen.
        If that is the case, block everything to recover from it.'''
        if self.detect_fall():
            while self.fsm.current_state != 'NO_FALL':
                # block everything and run the recovery motion until the robot is back on its feet
                self.fsm.execute_action()
                self.robot.step(self.time_step)

            
    def avoid_line(self,img):
        self.img = img
        #print('image imported')
        if self.line_detection():
            #print('self_line_detection')

            self.fsm.execute_action()
            while self.fsm.current_state != 'NO_LINE':

                if self.current_motion.is_over():
                    self.current_motion.set(self.library.get('Stand'))
                    image_turning_around = self.get_image()
                    self.img = image_turning_around
                    self.line_detection()
                self.fsm.execute_action()
                self.robot.step(self.time_step)                
 

        else:
            #print('No line')
            pass

    def turn_around(self):
        self.current_motion.set(self.library.get('TurnLeft60'))
        self.fsm.transition_to('LINE_DETECTED')        




    def line_detection(self, ya_max = 30, yb_max = 30):

        image = self.img

        BGR = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)

        # changing to hsv for better processing for the colors
        hsv = cv2.cvtColor(BGR, cv2.COLOR_RGB2HSV)
        #plot_in_range(hsv)
        #if self.time_step > 1 and  self.time_step < 1.5:

        # masking lower and upper bounds BGR
        # These are for BGR
        lower = (103 ,127,157)
        upper = (179, 239, 255)
        #lower = (0, 127, 0)
        #upper = (103, 255, 255)
        # masking BGR new
        #upper = (103, 132, 255)
        #lower = (0, 0, 0)
      
        # changing threshold to BGR
        #lower = (80, 50, 0)
        #upper = (255, 255, 16)

        mask = cv2.inRange(hsv, lower, upper)
 
        '''
        if self.i == 10:
            self.plot_in_range(image)
            self.plot_in_range(hsv)
            self.plot_in_range(mask)
            print(mask)
        '''




        # edge detection ( this is essential since the hough transform only able to identify gray scale image)
        edge = cv2.Canny(mask, 50, 200, None, 3)


        #line = np.asarray(edge)

        lines = cv2.HoughLinesP(edge, 1, np.pi/180, 50, minLineLength=10, maxLineGap=4) 

        #print(lines)
        # Just y position indicator can show us the correct position
        ya_pixel = 0
        yb_pixel = 0 
        self.fsm.transition_to('NO_LINE')
        line_detection = False
        if lines is not None:
            number_of_lines = len(lines)
            for i in range(0, len(lines)):
                for j in range(0, len(lines[i])):
                    ya_pixel = ya_pixel + lines[i][j][1]
                    yb_pixel = ya_pixel + lines[i][j][3]
            if ya_pixel/number_of_lines > ya_max or yb_pixel/number_of_lines > yb_max:
                line_detection = True
                self.fsm.transition_to('LINE_DETECTED')

        return line_detection
        
            
    def detect_fall(self):
        '''Detect a fall from the accelerometer and update the FSM state.'''
        self.accelerometer.update_average()
        [acc_x, acc_y, _] = self.accelerometer.get_average()
        fall = False
        if acc_x < -7:
            self.fsm.transition_to('FRONT_FALL')
            fall = True
        elif acc_x > 7:
            self.fsm.transition_to('BACK_FALL')
            fall = True
        if acc_y < -7:
            # Fell to its right, pushing itself on its back
            self.RShoulderRoll.setPosition(-1.2)
            self.fsm.transition_to('SIDE_FALL')
            fall = True
        elif acc_y > 7:
            # Fell to its left, pushing itself on its back
            self.LShoulderRoll.setPosition(1.2)
            self.fsm.transition_to('SIDE_FALL')
            fall = True
        return fall


    def pending(self):
        '''Wait for the current motion to finish before going back to NO_FALL.'''
        if self.current_motion.is_over():
            self.current_motion.set(self.library.get('Stand'))
            self.fsm.transition_to('NO_FALL')

    def front_fall(self):
        self.current_motion.set(self.library.get('GetUpFront'))
        self.fsm.transition_to('BLOCKING_MOTION')

    def back_fall(self):
        self.current_motion.set(self.library.get('GetUpBack'))
        self.fsm.transition_to('BLOCKING_MOTION')

    def wait(self):
        pass
