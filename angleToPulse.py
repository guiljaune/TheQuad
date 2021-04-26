#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 16:27:16 2020

@author: linux-asd
"""
import numpy as np
import time

def limitpulse(pulse, max, min):
    if pulse > max:
        pulse = max
    elif pulse < min:
        pulse = min
    return pulse


def convert(FR_angles, FL_angles, BR_angles, BL_angles):
    # It might change for other servomotors.
    pulse = np.empty([12])
    #FR
    pulse[0] = int(315/90 * np.rad2deg(FR_angles[0]) + 515)
    pulse[1] = int(-315/90 * np.rad2deg(FR_angles[1]) + 515) 
    pulse[2] = int(225/90 * np.rad2deg(FR_angles[2]) + 550)
    #FL
    pulse[3] = int(315/90 * np.rad2deg(FL_angles[0]) + 515)
    pulse[4] = int(315/90 * np.rad2deg(FL_angles[1]) + 515) 
    pulse[5] = int(225/90 * np.rad2deg(FL_angles[2]) + 475)
    #BR
    pulse[6] = int(315/90 + np.rad2deg(BR_angles[0]) + 515)
    pulse[7] = int(-315/90 * np.rad2deg(BR_angles[1]) + 515) 
    pulse[8] = int(225/90 * np.rad2deg(BR_angles[2]) + 550)
    #BL
    pulse[9] = int(315/90 * np.rad2deg(BL_angles[0]) + 450)
    pulse[10] = int(315/90 * np.rad2deg(BL_angles[1]) + 515) 
    pulse[11] = int(225/90 * np.rad2deg(BL_angles[2]) + 475)

    #Check limits
    max_shoulder = 515 + 100
    min_shoulder = 515 - 100
    max_femur = 515 + 315
    min_femur = 515 - 315
    min_tibia = 515 - 315
    max_tibia = 515 + 315


    pulse[0] = limitpulse(pulse[0], max_shoulder, min_shoulder)
    pulse[3] = limitpulse(pulse[3], max_shoulder, min_shoulder)
    pulse[6] = limitpulse(pulse[6], max_shoulder, min_shoulder)
    pulse[9] = limitpulse(pulse[9], max_shoulder, min_shoulder)
 
    pulse[1] = limitpulse(pulse[1], max_femur, min_femur)
    pulse[4] = limitpulse(pulse[4], max_femur, min_femur)
    pulse[7] = limitpulse(pulse[7], max_femur, min_femur)
    pulse[10] = limitpulse(pulse[10], max_femur, min_femur)

    pulse[2] = limitpulse(pulse[2], max_tibia, min_tibia)
    pulse[5] = limitpulse(pulse[5], max_tibia, min_tibia)
    pulse[8] = limitpulse(pulse[8], max_tibia, min_tibia)
    pulse[11] = limitpulse(pulse[11], max_tibia, min_tibia)


    return pulse


