#!/usr/bin/env python
import numpy as np
from scipy.linalg import expm
import rospkg

from ur3_driver.msg import command
from ur3_driver.msg import position
from ur3_driver.msg import gripper_input

def Get_MS():
	# Initialize return values of M and S
	M = np.eye(4)
	S = np.zeros((6,6))

	S[0] = [0,0,1, 0.15, 0.15, 0]
	S[1] = [0,1,0, -0.162, 0, -0.150]
	S[2] = [0,1,0, -0.162, 0, 0.094]
	S[3] = [0,1,0, -0.162, 0, 0.307]
	S[4] = [1,0, 0, 0, 0.162, -0.26]
	S[5] = [0,1,0, -0.162, 0, 0.390]

	M[0] = [0, -1, 0, 0.390]
	M[1] = [0, 0, -1, 0.401]
	M[2] = [1, 0, 0, 0.2155]
	M[3] = [0, 0, 0, 1]
	
	return M, S


def lab_fk(theta1, theta2, theta3, theta4, theta5, theta6):
	# Initialize the return_value 
	return_value = [None, None, None, None, None, None]

	print("Foward kinematics calculated:\n")

	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	theta = np.array([theta1,theta2,theta3,theta4,theta5,theta6])
	T = np.eye(4)

	M, S = Get_MS()
	
	S_mat1 = np.array([[0, -S[0][2], S[0][1], S[0][3]], [S[0][2],0,-S[0][0], S[0][4]], [-S[0][1], S[0][0], 0, S[0][5]], [0,0,0,0]])
	exp1 = expm(S_mat1*theta1)
	
	S_mat2 = np.array([[0, -S[1][2], S[1][1], S[1][3]], [S[1][2],0,-S[1][0], S[1][4]], [-S[1][1], S[1][0], 0, S[1][5]], [0,0,0,0]])
	exp2 = expm(S_mat2*theta2)
	
	S_mat3 = np.array([[0, -S[2][2], S[2][1], S[2][3]], [S[2][2],0,-S[2][0], S[2][4]], [-S[2][1], S[2][0], 0, S[2][5]], [0,0,0,0]])
	exp3 = expm(S_mat3*theta3)

	S_mat4 = np.array([[0, -S[3][2], S[3][1], S[3][3]], [S[3][2],0,-S[3][0], S[3][4]], [-S[3][1], S[3][0], 0, S[3][5]], [0,0,0,0]])
	exp4 = expm(S_mat4*theta4)

	S_mat5 = np.array([[0, -S[4][2], S[4][1], S[4][3]], [S[4][2],0,-S[4][0], S[4][4]], [-S[4][1], S[4][0], 0, S[4][5]], [0,0,0,0]])
	exp5 = expm(S_mat5*theta5)

	S_mat6 = np.array([[0, -S[5][2], S[5][1], S[5][3]], [S[5][2],0,-S[5][0], S[5][4]], [-S[5][1], S[5][0], 0, S[5][5]], [0,0,0,0]])
	exp6 = expm(S_mat6*theta6)

	T = np.matmul(exp1, exp2)
	T = np.matmul(T, exp3)
	T = np.matmul(T, exp4)
	T = np.matmul(T, exp5)
	T = np.matmul(T, exp6)

	T = np.matmul(T, M)

	print(str(T) + "\n")

	return_value[0] = theta1 + np.pi
	return_value[1] = theta2
	return_value[2] = theta3
	return_value[3] = theta4 - (0.5*np.pi)
	return_value[4] = theta5
	return_value[5] = theta6

	return return_value


def inverse_kinematics(xWgrip, yWgrip, zWgrip, yaw_WgripDegree):
	return_value = np.array([0, 0, 0, 0, 0, 0])

	# Function that calculates an elbow up 
	# inverse kinematics solution for the UR3

	L = [.152, .120, .244, .093, .213, .083, .083, .082, .0535, .059]

	theta = yaw_WgripDegree

	x = xWgrip + .150
	y = yWgrip - .150
	z = zWgrip - .010
	theta = theta*(np.pi/180)

	x_cen = x-L[8]*np.cos(theta)
	y_cen = y-L[8]*np.sin(theta)
	z_cen = z

	theta_1 = np.arctan2(y_cen, x_cen) - np.arcsin((.027+L[5])/np.sqrt(y_cen**2 + x_cen**2))

	x_3end = x_cen - L[7]*np.cos(theta_1) + (L[5]+.027)*np.sin(theta_1)
	y_3end = y_cen - L[6]*np.sin(theta_1) - (L[5]+.027)*np.cos(theta_1)
	z_3end = z_cen + L[7] + L[9]

	x2 = np.sqrt(x_3end**2 + y_3end**2)
	y2 = z_3end - L[0]
	length = (x2**2 + y2**2 - L[2]**2 - L[4]**2)/(2*L[2]*L[4])

	theta_3 = np.arctan2(np.sqrt(1-length**2),length);
	theta_2 = -np.arctan2(y2, x2) - np.arctan2((L[4]*np.sin(theta_3)), (L[2] + L[4]*np.cos(theta_3)))

	theta_4 = theta_2 + theta_3
	theta_5 = -np.pi/2
	theta_6 = theta_1 + np.pi/2 - theta
	
	# print theta values (in degree) calculated from inverse kinematics
	print(str(theta_1*180/np.pi) + " " + str(theta_2*180/np.pi) + " " + \
			str(theta_3*180/np.pi) + " " + str(theta_4*180/np.pi) + " " + \
			str(theta_5*180/np.pi) + " " + str(theta_6*180/np.pi))

	# obtain return_value from forward kinematics function
	return_value = lab_fk(theta_1, theta_2, theta_3, theta_4, theta_5, theta_6)

	return return_value
