#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab3_header import *

"""
Use 'expm' for matrix exponential.
Angles are in radian, distance are in meters.
"""

def skew(w):
    return np.array([[0,      -w[2],  w[1]],
                     [w[2],       0, -w[0]],
                     [-w[1], w[0],       0]])

def VecTose3(V):
    w_skew = skew([V[0], V[1], V[2]])
    v = [V[3], V[4], V[5]]
    return np.r_[np.c_[w_skew, v],
                 np.zeros((1, 4))]

def Get_MS():
	# =================== Your code starts here ====================#
	# Fill in the correct values for S1~6, as well as the M matrix

	w1 = np.array([0,0,1]); q1 = np.array([-150,150,10]);
	w2 = np.array([0,1,0]); q2 = np.array([-150,270,162]);
	w3 = np.array([0,1,0]); q3 = np.array([94,270,162]);
	w4 = np.array([0,1,0]); q4 = np.array([307,177,162]);
	w5 = np.array([1,0,0]); q5 = np.array([307,260,162]);
	w6 = np.array([0,1,0]); q6 = np.array([390,260,162]);

	v1 = -1*np.cross(w1,q1)
	v2 = -1*np.cross(w2,q2)
	v3 = -1*np.cross(w3,q3)
	v4 = -1*np.cross(w4,q4)
	v5 = -1*np.cross(w5,q5)
	v6 = -1*np.cross(w6,q6)

	S  = np.zeros((6,6))
	S[0] = np.concatenate((w1, v1), axis=None)
	S[1] = np.concatenate((w2, v2), axis=None)
	S[2] = np.concatenate((w3, v3), axis=None)
	S[3] = np.concatenate((w4, v4), axis=None)
	S[4] = np.concatenate((w5, v5), axis=None)
	S[5] = np.concatenate((w5, v5), axis=None)

	M  = np.array([[0,-1, 0, 390  ],
				   [0, 0,-1, 401  ],
				   [1, 0, 0, 215.5],
				   [0, 0, 0, 1    ]]);
	print('S', S)
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
	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)
	M, S = Get_MS()
	for i in range(6):
		T = np.matmul(T, expm(VecTose3(S[i]*theta[i])))
	T = np.matmul(T, M)/1000

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value
