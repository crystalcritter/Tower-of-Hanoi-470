#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from math import pi
import math

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix
	M = np.array([[0,-1,0,0.392],[0,0,-1,0.432],[1,0,0,0.2155],[0,0,0,1]])
	S = np.array([[0,0,1,0.150,0.150,0],[0,1,0,-0.162,0,-0.150],[0,1,0,-0.162,0,0.094],[0,1,0,-0.162,0,0.307],[1,0,0,0,0.162,-0.281],[0,1,0,-0.162,0,0.392]])
 

	# ==============================================================#
	return M, S


"""
Function that calculates encoder numbers for each motor
"""
def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):

	# Initialize the return_value
	return_value = [None, None, None, None, None, None]

	# =========== Implement joint angle to encoder expressions here ===========
	print("Foward kinematics calculated:\n")

	# =================== Your code starts here ====================#
	M , S = Get_MS()
	T = np.eye(4)
	theta = np.array([theta1, theta2, theta3, theta4, theta5, theta6])


	for i in range(6):
		wx,wy,wz,vx,vy,vz = S[i,:]
		S_hat = np.array([[0,-wz,wy,vx], [wz,0,-wx,vy],[-wy,wx,0,vz],[0,0,0,0]])
		T = T@ expm(S_hat*theta[i])
	#T = expm(S[0]*theta1)@expm(S[1]*theta2)@expm(S[2]*theta3)@expm(S[3]*theta4)@expm(S[4]*theta5)@expm(S[5]*theta6)@M
	# ==============================================================#
	T = T@M
	print(str(T) + "\n")

	return_value[0] = theta1 + np.pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*np.pi)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value




def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
    yaw_WgripDegree = np.radians(yaw_WgripDegree)

    L1 = 0.152
    L2 = 0.120
    L3 = 0.244
    L4 = 0.093
    L5 = 0.213
    L6 = 0.104
    L7 = 0.085
    L8 = 0.092
    L9 = 0.0535
    L10 = 0.059

    theta5 = -np.pi/2

    xoffset = 0.15
    yoffset = 0.15
    zoffset = 0.01
    xCen = xWgrip - L9*np.cos(yaw_WgripDegree) + xoffset
    yCen = yWgrip - L9*np.sin(yaw_WgripDegree) - yoffset
    zCen = zWgrip - zoffset

    #CHANGED THIS LITTLE CHUNK HERE arcsin d/r is the little triangle you make from gripper to center
    d = L2 - L4 + L6 
    r = np.hypot(xCen, yCen) #hypotenuse function
    print(d/r)
    theta1 = np.arctan2(yCen, xCen) - np.arcsin(d/r)

    theta6 = theta1 + np.pi/2 - yaw_WgripDegree

    xEnd = xCen - L7*np.cos(theta1) + L6*np.sin(theta1) + 0.027*np.sin(theta1)
    yEnd = yCen - (0.027*np.cos(theta1) + L6*np.cos(theta1) + L7*np.sin(theta1))
    zEnd = zCen + L10 + L8

    # 6. Side view: planar 2-link problem (cosine rule), elbow-up
    R = np.hypot(xEnd, yEnd)  # horizontal joint 2
    h = zEnd - L1                                     # height above joint 2
    D = np.hypot(R, h)

    #law of cosines here
    cos_elbow = (L3**2 + L5**2 - D**2) / (2*L3*L5)
    theta3 = np.pi - np.arccos(cos_elbow)

    cos_psi = (L3**2 + D**2 - L5**2) / (2*L3*D)
    psi = np.arccos(cos_psi)
    theta2 = -(np.arctan2(h, R) + psi)

    theta4 = - theta2 - theta3
    
    print(theta1, theta2, theta3, theta4, theta5, theta6)
    return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
