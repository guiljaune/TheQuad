#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 11 16:38:15 2020

@author: miguel-asd
"""
import time
import numpy as np
import matplotlib.pyplot as plt




#function neccesary to build a parametrized bezier curve
def f(n,k): #calculates binomial factor (n k)
    return np.math.factorial(n)/(np.math.factorial(k)*np.math.factorial(n-k))

def b(t,k,point):
    n = 9 #10 points bezier curve
    return point*f(n,k)*np.power(t,k)*np.power(1-t,n-k)


#gait planner in order to move all feet
class trotGait2:
    def __init__(self):
        self.bodytoFeet = np.zeros([4,3])
        self.phi = 0.
        self.phiStance = 0.
        self.lastTime = 0.
        self.alpha = 0.
        self.s = False
        self.pos = np.array([0, 0, 0], dtype=float)

    """This trajectory planning is mostly based on: 
    https://www.researchgate.net/publication/332374021_Leg_Trajectory_Planning_for_Quadruped_Robots_with_High-Speed_Trot_Gait"""
    def calculateStance2(self , phi_st , V):#phi_st between [0,1), angle in degrees
        p_stance = (1-2*phi_st)

        stanceX = -p_stance*np.abs(V)+0.02
        stanceY = 0 #-p_stance*np.abs(V)
        stanceZ = 0

        return stanceX, stanceY , stanceZ


    def calculateBezier_swing2(self , phi_sw , V , stepH):#phi between [0,1), angle in degrees

        X = -np.abs(V) * 20 * np.array([-0.05 ,
                                  -0.06 ,
                                  -0.07 , 
                                  -0.07 ,
                                  0. ,
                                  0. , 
                                  0.07 ,
                                  0.07 ,
                                  0.06 ,
                                  0.05 ])
    
        Y = 0 * np.abs(V) * np.array([ 0.05 ,
                                   0.06 ,
                                   0.07 , 
                                   0.07 ,
                                   0. ,
                                   -0. , 
                                   -0.07 ,
                                   -0.07 ,
                                   -0.06 ,
                                   -0.05 ])
    

        Z = stepH*20*np.array([0.,
                                      0.,
                                      0.04,
                                      0.04,
                                      0.04,
                                      0.05,
                                      0.05,
                                      0.05,
                                      0.,
                                      0.])
        swingX = 0.
        swingY = 0.
        swingZ = 0.
        for i in range(10): #sum all terms of the curve
            swingX = swingX + b(phi_sw,i,X[i])
            swingY = swingY + b(phi_sw,i,Y[i])
            swingZ = swingZ + b(phi_sw,i,Z[i])

        return swingX, swingY , swingZ


    def stepTrajectory2(self , phi , V , stepH, centerToFoot): #phi belong [0,1), angles in degrees
        if (phi >= 1):
            phi = phi - 1.
        

        stepOffset = 0.5
        if phi <= stepOffset: #stance phase
            phiStance = phi/stepOffset
            stepX_long , stepY_long , stepZ_long = self.calculateStance2(phiStance , V )#longitudinal step
        #            print(phi,phiStance, stepX_long)
        else: #swing phase
            phiSwing = (phi-stepOffset)/(1-stepOffset)
            stepX_long , stepY_long , stepZ_long = self.calculateBezier_swing2(phiSwing , V , stepH)#longitudinal step


        coord = np.empty(3)
        coord[0] = stepX_long 
        coord[1] = stepY_long
        coord[2] = stepZ_long 

        return coord


        #computes step trajectory for every foot, defining L which is like velocity command, its angle,
    #offset between each foot, period of time of each step and the initial vector from center of robot to feet.
    def loop2(self , V , T , offset , stepH, bodytoFeet_ ):

        if T <= 0.01:
            T = 0.01

        if (self.phi >= 0.99):
            self.lastTime= time.time()
        self.phi = (time.time()-self.lastTime)/T
        print(self.phi)
        #now it calculates step trajectory for every foot
        step_coord = self.stepTrajectory2(self.phi + offset[0] , V , stepH, np.squeeze(np.asarray(bodytoFeet_[0,:]))) #FR
        self.bodytoFeet[0,0] =  bodytoFeet_[0,0] - step_coord[0]
        self.bodytoFeet[0,1] =  bodytoFeet_[0,1] - step_coord[1]
        self.bodytoFeet[0,2] =  bodytoFeet_[0,2] + step_coord[2]

        step_coord = self.stepTrajectory2(self.phi + offset[1] , V , stepH, np.squeeze(np.asarray(bodytoFeet_[1,:])))#FL
        self.bodytoFeet[1,0] =  bodytoFeet_[1,0] + step_coord[0]
        self.bodytoFeet[1,1] =  bodytoFeet_[1,1] + step_coord[1]
        self.bodytoFeet[1,2] =  bodytoFeet_[1,2] + step_coord[2]

        step_coord = self.stepTrajectory2(self.phi + offset[2] , V , stepH, np.squeeze(np.asarray(bodytoFeet_[2,:])))#BR
        self.bodytoFeet[2,0] =  bodytoFeet_[2,0] - step_coord[0]
        self.bodytoFeet[2,1] =  bodytoFeet_[2,1] - step_coord[1]
        self.bodytoFeet[2,2] =  bodytoFeet_[2,2] + step_coord[2]

        step_coord = self.stepTrajectory2(self.phi + offset[3] , V , stepH, np.squeeze(np.asarray(bodytoFeet_[3,:])))#BL
        self.bodytoFeet[3,0] =  bodytoFeet_[3,0] + step_coord[0]
        self.bodytoFeet[3,1] =  bodytoFeet_[3,1] + step_coord[1]
        self.bodytoFeet[3,2] =  bodytoFeet_[3,2] + step_coord[2]
        #

        return self.bodytoFeet



if __name__ == "__main__":
    from mpl_toolkits.mplot3d import Axes3D
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    x_list = []
    y_list = []
    z_list = []
    vitesse = 0.05
    direction =0
    stepH = 0.05

    for p in range(100):
        t = (p+1) / 100

        s_Offset = 0.5
        trot = trotGait2()


        if t <= s_Offset:
            tStance = t/s_Offset
            x, y, z = trot.calculateStance2(tStance, vitesse)
        else:
            tSwing = (t-s_Offset)/(1-s_Offset)
            x, y, z = trot.calculateBezier_swing2(tSwing, vitesse, stepH)

        x_list.append(x)
        y_list.append(y)
        z_list.append(z)


    ax.plot3D(x_list, y_list, z_list)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')



    plt.show()



