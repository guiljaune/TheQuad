#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 16:27:16 2020

@author: linux-asd
"""
import numpy as np
import time





def convert(FR_angles, FL_angles, BR_angles, BL_angles):
    # It might change for other servomotors.
    pulse = np.empty([12])
    #FR
    pulse[0] = int(325/90 * np.rad2deg(BL_angles[0]) + 900)
    pulse[1] = int(325/90 * np.rad2deg(BL_angles[1]) + 270) 
    pulse[2] = int(225/90 * np.rad2deg(BL_angles[2]) + 450)
    #FL
    pulse[3] = int(65/90 + np.rad2deg(BL_angles[0]) + 65)
    pulse[4] = int(63.5/90 * np.rad2deg(BL_angles[1])) 
    pulse[5] = int(45/90 * np.rad2deg(BL_angles[2]) + 55)
    #BR
    pulse[6] = int(65/90 + np.rad2deg(BL_angles[0]) + 65)
    pulse[7] = int(63.5/90 * np.rad2deg(BL_angles[1])) 
    pulse[8] = int(45/90 * np.rad2deg(BL_angles[2]) + 55)
    #BL
    pulse[9] = int(325/90 * np.rad2deg(BL_angles[0]) + 900)
    pulse[10] = int(325/90 * np.rad2deg(BL_angles[1]) + 270) 
    pulse[11] = int(225/90 * np.rad2deg(BL_angles[2]) + 450)


    if pulse[11] > 800:
        pulse[11] = 800
    elif pulse[11] < 270:
        pulse[11] = 270

    if pulse[10] > 500:
        pulse[10] = 500
    elif pulse[10] < 200:
        pulse[10] = 200

    if pulse[9] > 1000:
        pulse[9] = 1000
    elif pulse[9] < 800:
        pulse[9] = 800


    return pulse


