#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Mar 15 20:31:07 2020

@author: miguel-asd
"""


import numpy as np
import time

import angleToPulse
from kinematic_model import robotKinematics
from gaitplanner2 import trotGait2


import Adafruit_PCA9685


pwm = Adafruit_PCA9685.PCA9685()

#servo addresses
fright_shoulder = 12
fright_femur = 13
fright_tibia = 14

flleft_shoulder = 4
flleft_femur = 5
flleft_tibia = 6

bright_shoulder = 8
bright_femur = 9
bright_tibia = 10

blleft_shoulder = 0   
blleft_femur = 1 
blleft_tibia = 2

# Set frequency to 60hz, good for servos.
pwm.set_pwm_freq(120)


robotKinematics = robotKinematics()

trot = trotGait2()
#robot properties
"""initial safe position"""
#angles
targetAngs = np.array([0 , np.pi/4 , -np.pi/2, 0 ,#BR
                       0 , np.pi/4 , -np.pi/2, 0 ,#BL
                       0 , np.pi/4 , -np.pi/2, 0 ,#FL
                       0 , np.pi/4 , -np.pi/2, 0 ])#FR



"initial foot position"
#foot separation (0.182 -> tetta=0) and distance to floor
Ydist = 0.20
Xdist = 0.2
height = 0.205
#body frame to foot frame vector
bodytoFeet0 = np.matrix([[ Xdist/2 , -Ydist/2 , -height],
                         [ Xdist/2 ,  Ydist/2 , -height],
                         [-Xdist/2 , -Ydist/2 , -height],
                         [-Xdist/2 ,  Ydist/2 , -height]])

orn = np.array([0, 0, 0])
pos = np.array([0, 0, 0])
lastTime = 0



T = 0.4 #period of time (in seconds) of every step
offset = np.array([0 , 0 , 0 , 0]) #defines the offset between each foot step in this order (FR,FL,BR,BL)

while(True):
    loopTime = time.time()


    #commandCoM , V , angle , Wrot , T  , compliantMode , yaw , pitch  = joystick.read()

    #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
    #bodytoFeet = trot.loop(L , angle , Lrot , T , offset , stepL, stepH, bodytoFeet0)
    #bodytoFeet = trot.loop(L , T , offset , stepH, bodytoFeet0)
    bodytoFeet = trot.loop2(0.0, 2, offset, 0.09, bodytoFeet0)
    #bodytoFeet = trot.loop2(0, 2, offset, 0, bodytoFeet0)



    #pos[0] = pidX(realPitch)
    #pos[1] = pidY(realRoll)


    #####################################################################################
    #####   kinematics Model: Input body orientation, deviation and foot position    ####
    #####   and get the angles, neccesary to reach that position, for every joint    ####
    FR_angles, FL_angles, BR_angles, BL_angles , transformedBodytoFeet = robotKinematics.solve(orn, pos, bodytoFeet)
    pulses = angleToPulse.convert(FR_angles, FL_angles, BR_angles, BL_angles)
    #print(pulses)

    pwm.set_pwm(fright_shoulder, 0, int(pulses[0]))
    pwm.set_pwm(flleft_shoulder, 0, int(pulses[3]))
    pwm.set_pwm(bright_shoulder, 0, int(pulses[6]))
    pwm.set_pwm(blleft_shoulder, 0, int(pulses[9]))

    pwm.set_pwm(fright_femur, 0, int(pulses[1]))
    pwm.set_pwm(flleft_femur, 0, int(pulses[4]))
    pwm.set_pwm(bright_femur, 0, int(pulses[7]))
    pwm.set_pwm(blleft_femur, 0, int(pulses[10]))

    pwm.set_pwm(fright_tibia, 0, int(pulses[2]))
    pwm.set_pwm(flleft_tibia, 0, int(pulses[5]))
    pwm.set_pwm(bright_tibia, 0, int(pulses[8]))
    pwm.set_pwm(blleft_tibia, 0, int(pulses[11]))


    # pwm.set_pwm(fright_shoulder, 0, 515)
    # pwm.set_pwm(flleft_shoulder, 0, 515)
    # pwm.set_pwm(bright_shoulder, 0, 515)
    # pwm.set_pwm(blleft_shoulder, 0, 450)

    # pwm.set_pwm(fright_femur, 0, 515)
    # pwm.set_pwm(flleft_femur, 0, 515)
    # pwm.set_pwm(bright_femur, 0, 515)
    # pwm.set_pwm(blleft_femur, 0, 515)

    # pwm.set_pwm(fright_tibia, 0, 515)
    # pwm.set_pwm(flleft_tibia, 0, 515)
    # pwm.set_pwm(bright_tibia, 0, 515)
    # pwm.set_pwm(blleft_tibia, 0, 515)



