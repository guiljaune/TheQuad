#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 20 18:20:45 2020

@author: linux-asd
"""
import numpy

def checkdomain(D):
    if D > 1 or D < -1:
        print("____OUT OF DOMAIN____")
        if D > 1:
            D = 0.99
            return D
        elif D < -1:
            D = -0.99
            return D
    else:
        return D
#this is based on this paper:
#"https://www.researchgate.net/publication/320307716_Inverse_Kinematic_Analysis_Of_A_Quadruped_Robot"
"""
"using pybullet frame"
"  z                     "
"    |                   "
"    |                   "
"    |    /  y           "
"    |   /               "
"    |  /                "
"    | /                 "
"    |/____________  x       "
"""
#IK equations now written in pybullet frame.
def solve_R(coord , coxa , femur , tibia):
    x4 = - coord[1]
    y4 = coord[2]
    z4 = - coord[0]
    l1 = coxa
    l2 = femur
    l3 = tibia
    D = (x4 ** 2 + y4 ** 2 - l1 ** 2 + z4 ** 2 - l2 ** 2 - l3 ** 2 ) / (2 * l2 * l3)
    D = checkdomain(D)
    teta1 =  - numpy.arctan2(-y4, x4) - numpy.arctan2(numpy.sqrt( x4 ** 2 + y4 ** 2 - l1 ** 2), - l1) + numpy.pi
    teta3 = numpy.arctan2(-numpy.sqrt(1 - D ** 2), D)
    teta2 = numpy.arctan2(z4, numpy.sqrt(x4 ** 2 + y4 ** 2  - l1 ** 2)) - numpy.arctan2(l3 *numpy.sin(teta3), l2 + l3 * numpy.cos(teta3))
    angles = numpy.array([teta1, teta2, teta3])

    # D = (coord[1]**2+(-coord[2])**2-coxa**2+(-coord[0])**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    # D = checkdomain(D)
    # gamma = numpy.arctan2(-numpy.sqrt(1-D**2),D)
    # tetta = -numpy.arctan2(coord[2],coord[1])-numpy.arctan2(numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),-coxa)
    # alpha = numpy.arctan2(-coord[0],numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    # angles = numpy.array([-tetta, alpha, gamma])
    return angles

def solve_L(coord , coxa , femur , tibia):
    x4 = coord[1]
    y4 = coord[2]
    z4 = - coord[0]
    l1 = coxa
    l2 = femur
    l3 = tibia
    D = (x4 ** 2 + y4 ** 2 - l1 ** 2 + z4 ** 2 - l2 ** 2 - l3 ** 2 ) / (2 * l2 * l3)
    D = checkdomain(D)
    teta1 =  - numpy.arctan2(-y4, x4) - numpy.arctan2(numpy.sqrt( x4 ** 2 + y4 ** 2 - l1 ** 2), - l1) + numpy.pi
    teta3 = numpy.arctan2(numpy.sqrt(1 - D ** 2), D)
    teta2 = numpy.arctan2(z4, numpy.sqrt(x4 ** 2 + y4 ** 2  - l1 ** 2)) - numpy.arctan2(l3 *numpy.sin(teta3), l2 + l3 * numpy.cos(teta3))
    angles = numpy.array([teta1, -teta2, teta3])


    # D = (coord[1]**2+(-coord[2])**2-coxa**2+(-coord[0])**2-femur**2-tibia**2)/(2*tibia*femur)  #siempre <1
    # D = checkdomain(D)
    # gamma = numpy.arctan2(-numpy.sqrt(1-D**2),D)
    # tetta = -numpy.arctan2(coord[2],coord[1])-numpy.arctan2(numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2),coxa)
    # alpha = numpy.arctan2(-coord[0],numpy.sqrt(coord[1]**2+(-coord[2])**2-coxa**2))-numpy.arctan2(tibia*numpy.sin(gamma),femur+tibia*numpy.cos(gamma))
    # angles = numpy.array([-tetta, alpha, gamma])
    return angles

