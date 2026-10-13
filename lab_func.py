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
	M = np.eye(4)
	S = np.zeros((6,6))
 

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

	T = [ [1.0, 0.0, 0.0], \
      [0.0, 1.0, 0.0], \
      [0.0, 0.0, 1.0] ]
	# ==============================================================#

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
	yaw_WgripDegree = np.radians(yaw_WgripDegree)
	theta1 = 0.0
	theta2 = 0.0
	theta3 = 0.0
	theta4 = 0.0
	theta5 = -90
	theta6 = 0.0

	xoffset = -0.15
	yoffset = 0.15
	zoffset = 0.01
	L1 = 0.152
	L2 = 0.12
	L3 = 0.244
	L4 = 0.093
	L5 = 0.213
	L6 = 0.104
	L7 = 0.085 
	L8 = 0.092
	L9 = 0.0535
	L10 = 0.059

	xCen = xWgrip - L9*np.sin(yaw_WgripDegree) + xoffset
	yCen = yWgrip - L9*np.cos(yaw_WgripDegree) - yoffset
	zCen = zWgrip - zoffset

	t1 = np.symbols('t1')
	t1 = np.atan2(yCen - (0.027*np.cos(t1) + L8*np.cos(t1) + L9*np.sin(t1)), xCen - L9 - L7)
	#bc t1 = atan2(yend,xend), one unknown
	eq0 = np.atan2(yCen - (0.027*np.cos(t1) + L8*np.cos(t1) + L9*np.sin(t1)), xCen - L9 - L7) - t1
	answer0 = np.linalg.solve(eq0, t1)
	theta1 = answer0[0]

	theta6 = theta1 + np.pi/2 - yaw_WgripDegree

	xEnd = xCen - L9 - L7 
	yEnd = yCen - (0.027*np.cos(theta1) + L8*np.cos(theta1) + L9*np.sin(theta1))
	zEnd = zCen - L10 - L8

	#theta2, theta3, theta4 calculations
	t2, t3 = symbols('t2, t3')
	xEnd2 = L3*np.cos(-t2) + L5*np.cos(t3)
	zEnd2 = L1 + L3*np.sin(-t2) - L5*np.sin(t3)
	eq1 = xEnd - xEnd2
	eq2 = zEnd - zEnd2
	answers = np.linalg.solve([eq1,eq2], [t2,t3])
	theta2 = answers[0]
	theta3 = answers[1]
	theta4 = np.pi/2 - 2*np.pi - theta2 + (np.pi-theta3)

	#xCen = xoffset + L3*np.cos(-theta2) + L5*np.cos(theta3) + L7
	#yCen = yoffset + L2*cos(theta1) + L3*np.sin(theta1) -L4*np.cos(theta1)
	#zCen = zoffset + L1 + L3*np.cos(-theta2) - L5*np.sin(theta3) - L8 - L10

	
	# ==============================================================#
	return lab_fk(theta1, theta2, theta3, theta4, theta5, theta6)
