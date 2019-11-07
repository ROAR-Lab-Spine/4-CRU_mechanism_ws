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

# tips: We can use ipython to test things around! (remeber to >> from tf import transformations)

class Robot_4CRU(object):
	"""class of Robot_4CRU"""
	def __init__(self):
		super(Robot_4CRU, self).__init__()

		self.init_pose = tfs.identity_matrix()
		
		# End-effector joint axis distribution angles
		self.alphas = np.sort(np.array([np.pi/4.0, np.pi/4.0 + np.pi/8.0, np.pi/2.0, np.pi/2.0 + np.pi/8.0, np.pi/2.0 + np.pi/4.0])/2.0)
		# Base joint axis distribution angles
		self.betas = np.sort(np.array([np.pi/4.0, np.pi/4.0 + np.pi/8.0]))
		self.eeff_diag_lengths = np.sort(np.array([6.25/np.cos(np.pi/8)-2.0, 6.25/np.cos(np.pi/8)]))
		self.base_diag_lengths = np.sort(np.array([	self.eeff_diag_lengths[0],
			self.eeff_diag_lengths[1],
			self.eeff_diag_lengths[0]*np.cos(self.alphas[1])/np.cos(self.betas[0]), 
			self.eeff_diag_lengths[1]*np.cos(self.alphas[0])/np.cos(self.betas[0]),
			self.eeff_diag_lengths[1]*np.cos(self.alphas[1])/np.cos(self.betas[0])	]))

		self.geometric_indcs = [2, 0, 1, 3]; # selected alpha, beta, eeff_diag_length, base_diag_length
		a = self.base_diag_lengths[self.geometric_indcs[3]]/(2.0*np.cos(self.betas[self.geometric_indcs[1]]))
		b = self.base_diag_lengths[self.geometric_indcs[3]]/(2.0*np.sin(self.betas[self.geometric_indcs[1]]))
		c = self.base_diag_lengths[self.geometric_indcs[2]]/(2.0*np.cos(self.alphas[self.geometric_indcs[0]]))
		d = self.base_diag_lengths[self.geometric_indcs[2]]/(2.0*np.sin(self.alphas[self.geometric_indcs[0]]))
		self.geometric_params = np.array([a, b, c, d])
		print self.geometric_params
		self.test_current_geometric_cond()

		self.inverse_kinematics()

	def test_current_geometric_cond(self):
		condition_names = ["Generic", "A", "B", "C"]
		curr_a = self.geometric_params[0]
		curr_b = self.geometric_params[1]
		curr_c = self.geometric_params[2]
		curr_d = self.geometric_params[3]
		if (curr_a < curr_c and curr_b > curr_d) or (curr_a > curr_c and curr_b < curr_d):
			print condition_names[0]
		elif curr_a == curr_c and curr_b != curr_d:
			print condition_names[1]
		elif curr_a != curr_c and curr_b == curr_d:
			print condition_names[2]
		elif curr_a == curr_c and curr_b == curr_d:
			print condition_names[3]
		else:
			print "Additional Mode is not achievable"

	def update_geometric_cond(self, new_geometric_indcs):
		self.geometric_indcs = new_geometric_indcs

	def inverse_kinematics(self):
		print "Hello World!, this is 4-CRU mechanism"

def main():
	rospy.init_node('robot_4cru', anonymous=True)
	robot = Robot_4CRU()
	print "Spinning..."
	rospy.spin()

if __name__ == '__main__':
	main()
		
		
