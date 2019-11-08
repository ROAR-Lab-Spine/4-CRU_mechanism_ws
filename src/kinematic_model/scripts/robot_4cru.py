#!/usr/bin/env python

# ROS environment
import rospy
import tf 
from tf import transformations as tfs 
# See the library at https://github.com/ROAR-Lab-Spine/geometry/blob/melodic-devel/tf/src/tf/transformations.py
import time

# from robot_sim.srv import RobotAction
# from robot_sim.srv import RobotActionRequest
# from robot_sim.srv import RobotActionResponse
# from robot_sim.msg import RobotState

import numpy as np
import math
from sympy import * # symbolic calculation for IK

# tips: We can use ipython to test things around! (remeber to >> from tf import transformations)

class Robot_4CRU(object):
	"""class of Robot_4CRU"""
	def __init__(self):
		super(Robot_4CRU, self).__init__()

		init_pose = tfs.identity_matrix()
		self.robot_pose = init_pose
		self.operation_mode = "H1"
		
		# End-effector joint axis distribution angles (5 cases)
		self.alphas = np.sort(np.array([np.pi/4.0, np.pi/4.0 + np.pi/8.0, np.pi/2.0, np.pi/2.0 + np.pi/8.0, np.pi/2.0 + np.pi/4.0])/2.0)
		# Base joint axis distribution angles (2 cases)
		self.betas = np.sort(np.array([np.pi/4.0, np.pi/4.0 + np.pi/8.0]))
		# end effector diagonal lengths (2 cases)
		self.eeff_diag_lengths = np.sort(np.array([6.25/np.cos(np.pi/8)-2.0, 6.25/np.cos(np.pi/8)]))
		# base diagonal length (5 cases)
		self.base_diag_lengths = np.sort(np.array([	self.eeff_diag_lengths[0],
			self.eeff_diag_lengths[1],
			self.eeff_diag_lengths[0]*np.cos(self.alphas[1])/np.cos(self.betas[0]), 
			self.eeff_diag_lengths[1]*np.cos(self.alphas[0])/np.cos(self.betas[0]),
			self.eeff_diag_lengths[1]*np.cos(self.alphas[1])/np.cos(self.betas[0])	]))

		self.set_geometric_params([2, 0, 1, 3])

	def set_geometric_params(self, new_geometric_indcs):
		self.geometric_indcs = new_geometric_indcs
		self.a = self.base_diag_lengths[self.geometric_indcs[3]]/(2.0*np.cos(self.betas[self.geometric_indcs[1]]))
		self.b = self.base_diag_lengths[self.geometric_indcs[3]]/(2.0*np.sin(self.betas[self.geometric_indcs[1]]))
		self.c = self.base_diag_lengths[self.geometric_indcs[2]]/(2.0*np.cos(self.alphas[self.geometric_indcs[0]]))
		self.d = self.base_diag_lengths[self.geometric_indcs[2]]/(2.0*np.sin(self.alphas[self.geometric_indcs[0]]))
		self.update_geometric_cond()

	def update_geometric_cond(self):
		curr_a = self.a
		curr_b = self.b
		curr_c = self.c
		curr_d = self.d
		if (curr_a < curr_c and curr_b > curr_d) or (curr_a > curr_c and curr_b < curr_d):
			self.geometric_cond = "Generic"
		elif curr_a == curr_c and curr_b != curr_d:
			self.geometric_cond = "A"
		elif curr_a != curr_c and curr_b == curr_d:
			self.geometric_cond = "B"
		elif curr_a == curr_c and curr_b == curr_d:
			self.geometric_cond = "C"
		else:
			self.geometric_cond = "Arbitrary"
			print "Additional Mode is not achievable"

	def inverse_kinematics(self, pose, operation_mode):
		print "Hello World!, this is 4-CRU mechanism"


def main():
	rospy.init_node('robot_4cru', anonymous=True)
	robot = Robot_4CRU()
	print "Spinning..."
	rospy.spin()

if __name__ == '__main__':
	main()
		
		
