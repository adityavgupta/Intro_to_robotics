#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
from lab5_header import *
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
	# Fill in the correct values for a1~6 and q1~6, as well as the M matrix

	w1 = np.array([0,0,1]); q1 = np.array([-.150,.150,.010]);
	w2 = np.array([0,1,0]); q2 = np.array([-.150,.270,.162]);
	w3 = np.array([0,1,0]); q3 = np.array([.094,.270,.162]);
	w4 = np.array([0,1,0]); q4 = np.array([.307,.177,.162]);
	w5 = np.array([1,0,0]); q5 = np.array([.307,.260,.162]);
	w6 = np.array([0,1,0]); q6 = np.array([.390,.260,.162]);

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
	S[5] = np.concatenate((w6, v6), axis=None)

	M  = np.array([[0,-1, 0, .390  ],
				   [0, 0,-1, .401  ],
				   [1, 0, 0, .2155],
				   [0, 0, 0, 1    ]]);
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
	T = np.matmul(T, M)

	# ==============================================================#

	print(str(T) + "\n")

	return_value[0] = theta1 + PI
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*PI)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


"""
Function that calculates an elbow up Inverse Kinematic solution for the UR3
"""
def lab_invk(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):

    # theta1 to theta6
	thetas = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

	l01 = 0.152
	l02 = 0.120
	l03 = 0.244
	l04 = 0.093
	l05 = 0.213
	l06 = 0.083
	l07 = 0.083
	l08 = 0.082
	l09 = 0.0535
	l10 = 0.059   # thickness of aluminum plate is around 0.01
	yaw = np.radians(yaw_WgripDegree)
	xgrip = xWgrip+0.15
	ygrip = yWgrip-0.15
	zgrip = zWgrip-0.01

	xcen = xgrip-l09*np.cos(yaw)
	ycen = ygrip-l09*np.sin(yaw)
	zcen = zgrip
	# theta1
	distToCen = np.sqrt(xcen**2 + ycen**2)
	thetas[0] = np.arctan2(ycen, xcen) - np.arcsin((l06+.027)/distToCen)        # Default value Need to Change
	# theta6
	thetas[5] = thetas[0]+PI/2-yaw     # Default value Need to Change
	x3end = xcen - l07*np.cos(thetas[0])+(l06+0.027)*np.sin(thetas[0])
	y3end = ycen - l07*np.sin(thetas[0])-(l06+0.027)*np.cos(thetas[0])
	z3end = l08+l10+zcen
	print(y3end, z3end)

	d = z3end - l01
	R = np.linalg.norm(np.array([x3end,y3end,d])-np.array([0,0,0]))
	R = np.sqrt(x3end**2+y3end**2+d**2)


	alpha = np.arcsin(d/R)
	beta = np.arccos((R**2+l03**2-l05**2)/(2*l03*R))
	gamma = np.arccos((-R**2+l03**2+l05**2)/(2*l03*l05))
	#print(alpha,beta,gamma)
	thetas[1]= -(alpha + beta)     # Default value Need to Change
	thetas[2]= PI - gamma      # Default value Need to Change
	thetas[3]= -(PI+thetas[1]-gamma) # Default value Need to Change
	thetas[4]=-PI/2      # Default value Need to Change


	print("theta1 to theta6: " + str(thetas) + "\n")
	return lab_fk(float(thetas[0]), float(thetas[1]), float(thetas[2]), \
		          float(thetas[3]), float(thetas[4]), float(thetas[5]) )
