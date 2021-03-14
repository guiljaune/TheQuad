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
from serial_com import ArduinoSerial
from gaitplanner2 import trotGait2





robotKinematics = robotKinematics()
arduino = ArduinoSerial('COM3') #need to specify the serial port
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
height = 0.16
#body frame to foot frame vector
bodytoFeet0 = np.matrix([[ Xdist/2 , -Ydist/2 , -height],
                         [ Xdist/2 ,  Ydist/2 , -height],
                         [-Xdist/2 , -Ydist/2 , -height],
                         [-Xdist/2 ,  Ydist/2 , -height]])

orn = np.array([0, 0, 0])
pos = np.array([0, 0, 0])
lastTime = 0



T = 0.4 #period of time (in seconds) of every step
offset = np.array([0.5 , 0. , 0. , 0.5]) #defines the offset between each foot step in this order (FR,FL,BR,BL)

while(True):
    loopTime = time.time()


    #commandCoM , V , angle , Wrot , T  , compliantMode , yaw , pitch  = joystick.read()

    #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
    #bodytoFeet = trot.loop(L , angle , Lrot , T , offset , stepL, stepH, bodytoFeet0)
    bodytoFeet = trot.loop2(0.05, 0.5, offset, 0.04, bodytoFeet0)
    #bodytoFeet = trot.loop2(0, 2, offset, 0, bodytoFeet0)



    #arduinoLoopTime , Xacc , Yacc , realRoll , realPitch = arduino.serialRecive()#recive serial messages

    #pos[0] = pidX(realPitch)
    #pos[1] = pidY(realRoll)


    #####################################################################################
    #####   kinematics Model: Input body orientation, deviation and foot position    ####
    #####   and get the angles, neccesary to reach that position, for every joint    ####
    FR_angles, FL_angles, BR_angles, BL_angles , transformedBodytoFeet = robotKinematics.solve(orn, pos, bodytoFeet)
    #print(FR_angles, FL_angles, BR_angles, BL_angles)
    pulsesCommand = angleToPulse.convert(FR_angles, FL_angles, BR_angles, BL_angles)

    arduino.serialSend(pulsesCommand)#send serial command to arduino
    #time.sleep(0.050)
    print(pulsesCommand)
    #test_pi and from dell


    #print(time.time() - loopTime ,T)




