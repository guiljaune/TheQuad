#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 16:27:16 2020

@author: linux-asd
"""
import numpy as np


def convert(FR_angles, FL_angles, BR_angles, BL_angles):
    # It might change for other servomotors.
    pulse = np.empty([12])
    #FR
    pulse[0] = int(65/90 + np.rad2deg(BL_angles[0]) + 65)
    pulse[1] = int(63.5/90 * np.rad2deg(BL_angles[1])) 
    pulse[2] = int(45/90 * np.rad2deg(BL_angles[2]) + 55)
    #FL
    pulse[3] = int(65/90 + np.rad2deg(BL_angles[0]) + 65)
    pulse[4] = int(63.5/90 * np.rad2deg(BL_angles[1])) 
    pulse[5] = int(45/90 * np.rad2deg(BL_angles[2]) + 55)
    #BR
    pulse[6] = int(65/90 + np.rad2deg(BL_angles[0]) + 65)
    pulse[7] = int(63.5/90 * np.rad2deg(BL_angles[1])) 
    pulse[8] = int(45/90 * np.rad2deg(BL_angles[2]) + 55)
    #BL
    pulse[9] = int(7.55 * np.rad2deg(BL_angles[0]) + 1200)
    pulse[10] = int(7.55 * np.rad2deg(BL_angles[1])+ 500) 
    pulse[11] = int(5.95 * np.rad2deg(BL_angles[2]) + 915)
    
    if pulse[11] > 1700:
        pulse[11] = 1700
    elif pulse[11] < 915:
        pulse[11] = 915

    if pulse[10] > 1000:
        pulse[10] = 1000
    elif pulse[10] < 500:
        pulse[10] = 500

    if pulse[9] > 1400:
        pulse[9] = 1400
    elif pulse[9] < 1000:
        pulse[9] = 1000

    return pulse
