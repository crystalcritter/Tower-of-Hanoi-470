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
		wx,wy,wz,vx,vy,vz = S[:, i]
		S_hat = np.array([[0,-wz,wy,vx], [wz,0,-wx,vy],[-wy,wx,0,vz],[0,0,0,0]])
		T = T@ expm(S_hat*theta[i])
	#T = expm(S[0]*theta1)@expm(S[1]*theta2)@expm(S[2]*theta3)@expm(S[3]*theta4)@expm(S[4]*theta5)@expm(S[5]*theta6)@M
	# ==============================================================#
	T = T@M
	print(str(T) + "\n")

	return_value[0] = theta1 + pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*pi)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	# =================== Your code starts here ====================#
	
	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = 0.0
	theta6 = 0.0
	
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
