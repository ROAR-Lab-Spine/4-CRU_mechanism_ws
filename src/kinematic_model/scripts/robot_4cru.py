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

		# End-effector joint axis distribution angles (5 cases)
		self.alphas = np.sort(np.array([np.pi/4.0, np.pi/4.0 + np.pi/8.0, np.pi/2.0, np.pi/2.0 + np.pi/8.0, np.pi/2.0 + np.pi/4.0])/2.0)
		# Base joint axis distribution angles (2 cases)
		self.betas = np.sort(np.array([np.pi/4.0, np.pi/4.0 + np.pi/8.0]))
		# end effector diagonal lengths (2 cases) unit in mm
		self.eeff_diag_lengths = np.sort(np.array([6.25/np.cos(np.pi/8)-2.0, 6.25/np.cos(np.pi/8)]))*25.4
		# base diagonal length (5 cases) unit in mm
		self.base_diag_lengths = np.sort(np.array([	self.eeff_diag_lengths[0],
			self.eeff_diag_lengths[1],
			self.eeff_diag_lengths[0]*np.cos(self.alphas[1])/np.cos(self.betas[0]), 
			self.eeff_diag_lengths[1]*np.cos(self.alphas[0])/np.cos(self.betas[0]),
			self.eeff_diag_lengths[1]*np.cos(self.alphas[1])/np.cos(self.betas[0])	]))
		# length of the UU couple (from CAD) unit in mm
		self.r = (1.75*2 + 2.54)*10
		self.joint_pos_range = np.array([0, 200])
		self.h_offset = 

		# Calculate geometric parameters
		# self.set_geometric_params([2, 1, 1, 3])
		self.set_geometric_params([2, 0, 1, 3])

		# Initialize pose in Schoenflies mode
		self.operation_mode = "H1"
		self.robot_pose_4dof = np.array([0, 0, 150, 0.2])

		pose_base_to_eeff, X, Y, Z, x_0, x_1, x_2, x_3 = self.convert_pose_4dof_to_pose_and_7var(self.robot_pose_4dof)
		self.robot_pose = pose_base_to_eeff
		self.inverse_kinematics(self.robot_pose_4dof)

	def set_geometric_params(self, new_geometric_indcs):
		self.geometric_indcs = new_geometric_indcs
		self.a = self.base_diag_lengths[self.geometric_indcs[3]]/2.0*np.cos(self.betas[self.geometric_indcs[1]])
		self.b = self.base_diag_lengths[self.geometric_indcs[3]]/2.0*np.sin(self.betas[self.geometric_indcs[1]])
		self.c = self.eeff_diag_lengths[self.geometric_indcs[2]]/2.0*np.cos(self.alphas[self.geometric_indcs[0]])
		self.d = self.eeff_diag_lengths[self.geometric_indcs[2]]/2.0*np.sin(self.alphas[self.geometric_indcs[0]])
		self.update_geometric_cond()

	def update_geometric_cond(self):
		curr_a = self.a # magnitude only
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
			self.geometric_cond = "Other"
			print "Additional Mode is not achievable!"
		print "Geometric Condition: ", self.geometric_cond
		print "[a, b, c, d] = ", [self.a, self.b, self.c, self.d], " mm"
		print "r = ", self.r, " mm"

	def inverse_kinematics(self, pose_4dof):
		reals, discriminants, has_solution = self.check_ik_feasible(pose_4dof)
		joint_pos_sol = np.full((4), self.joint_pos_range[1]/2)
		print joint_pos
		print reals
		print discriminants
		print has_solution

		# choose closest h from the two +/- cases

		# check joint range from vertical offsets 

		# check the swivel angle limit on both ends of the U-U couplings

		return joint_pos_sol

	def check_ik_feasible(self, pose_4dof):
		pose_base_to_eeff, X, Y, Z, x_0, x_1, x_2, x_3 = self.convert_pose_4dof_to_pose_and_7var(pose_4dof)
		r = self.r
		
		reals = np.zeros([4])
		discriminants = np.zeros([4])
		has_solution = True

		for i in range(4):
			# convert signs based on joint indices
			if i == 0:
				a = self.a; b = -self.b; c = self.c; d = -self.d
			elif i == 1:
				a = self.a; b = self.b; c = self.c; d = self.d
			elif i == 2:
				a = -self.a; b = self.b; c = -self.c; d = self.d
			elif i == 3:
				a = -self.a; b = -self.b; c = -self.c; d = -self.d

			discriminants[i] = -X**2 + 2.0*X*a - 2.0*X*c*x_0**2 - 2.0*X*c*x_1**2 + \
			2.0*X*c*x_2**2 + 2.0*X*c*x_3**2 + 4.0*X*d*x_0*x_3 - 4.0*X*d*x_1*x_2 \
			- Y**2 + 2.0*Y*b - 4.0*Y*c*x_0*x_3 - 4.0*Y*c*x_1*x_2 - \
			2.0*Y*d*x_0**2 + 2.0*Y*d*x_1**2 - 2.0*Y*d*x_2**2 + 2.0*Y*d*x_3**2 - \
			a**2 + 2.0*a*c*x_0**2 + 2.0*a*c*x_1**2 - 2.0*a*c*x_2**2 - \
			2.0*a*c*x_3**2 - 4.0*a*d*x_0*x_3 + 4.0*a*d*x_1*x_2 - b**2 + \
			4.0*b*c*x_0*x_3 + 4.0*b*c*x_1*x_2 + 2.0*b*d*x_0**2 - 2.0*b*d*x_1**2 \
			+ 2.0*b*d*x_2**2 - 2.0*b*d*x_3**2 - c**2*x_0**4 - \
			2.0*c**2*x_0**2*x_1**2 + 2.0*c**2*x_0**2*x_2**2 - \
			2.0*c**2*x_0**2*x_3**2 - 8.0*c**2*x_0*x_1*x_2*x_3 - c**2*x_1**4 - \
			2.0*c**2*x_1**2*x_2**2 + 2.0*c**2*x_1**2*x_3**2 - c**2*x_2**4 - \
			2.0*c**2*x_2**2*x_3**2 - c**2*x_3**4 - 8.0*c*d*x_0**2*x_1*x_2 + \
			8.0*c*d*x_0*x_1**2*x_3 - 8.0*c*d*x_0*x_2**2*x_3 + \
			8.0*c*d*x_1*x_2*x_3**2 - d**2*x_0**4 + 2.0*d**2*x_0**2*x_1**2 - \
			2.0*d**2*x_0**2*x_2**2 - 2.0*d**2*x_0**2*x_3**2 + \
			8.0*d**2*x_0*x_1*x_2*x_3 - d**2*x_1**4 - 2.0*d**2*x_1**2*x_2**2 - \
			2.0*d**2*x_1**2*x_3**2 - d**2*x_2**4 + 2.0*d**2*x_2**2*x_3**2 - \
			d**2*x_3**4 + r**2  

			reals[i] = Z - 2.0*c*x_0*x_2 + 2.0*c*x_1*x_3 + 2.0*d*x_0*x_1 + 2.0*d*x_2*x_3

		if np.any(np.sign(discriminants) < 0):
			has_solution = False
		
		return reals, discriminants, has_solution

	def convert_pose_4dof_to_pose_and_7var(self, pose_4dof):
		if self.operation_mode == "H1":
			t_mat = tfs.translation_matrix((pose_4dof[0], pose_4dof[1], pose_4dof[2]))
			r_mat = tfs.rotation_matrix(pose_4dof[3], (0, 0, 1))
			pose_base_to_eeff = tfs.concatenate_matrices(t_mat, r_mat)
		else:
			pass

		X, Y, Z, x_0, x_1, x_2, x_3 = self.convert_pose_mat_to_7var(pose_base_to_eeff)

		return pose_base_to_eeff, X, Y, Z, x_0, x_1, x_2, x_3

	def convert_pose_mat_to_7var(self, pose_mat):
		quat = tfs.quaternion_from_matrix(pose_mat)
		xyz = tfs.translation_from_matrix(pose_mat)
		X = xyz[0]; Y = xyz[1]; Z = xyz[2] 
		x_0 = quat[3]; x_1 = quat[0]; x_2 = quat[1]; x_3 = quat[2]
		return X, Y, Z, x_0, x_1, x_2, x_3

def main():
	rospy.init_node('robot_4cru', anonymous=True)
	robot = Robot_4CRU()
	print "Spinning..."
	rospy.spin()

if __name__ == '__main__':
	main()
		
		
