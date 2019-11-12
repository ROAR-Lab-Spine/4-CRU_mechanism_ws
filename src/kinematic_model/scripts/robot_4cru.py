#!/usr/bin/env python

# ROS environment
import rospy
import tf 
from tf import transformations as tfs 
# See the library at https://github.com/ROAR-Lab-Spine/geometry/blob/melodic-devel/tf/src/tf/transformations.py
import time

from kinematic_model.srv import RobotIK, RobotIKRequest, RobotIKResponse
from geometry_msgs.msg import TransformStamped, PoseArray, Pose, Point, Quaternion
from trajectory_msgs.msg import JointTrajectoryPoint
from control_msgs.msg import JointJog
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

import numpy as np
import numpy.matlib
import numpy.linalg as la
import math
# from sympy import * # symbolic calculation for IK

# tips: We can use ipython to test things around! (remeber to >> from tf import transformations)
# Word Conventions
# 	tfm: tf.TransformerROS() using with TransformStamped
#	pose: geometry_msgs.msg.Pose()
#   tf_mat: numpy.matrix 4x4 representation of the transform
#   tfstamp = TransformStamped()

class Robot4CRU(object):
	"""class of Robot_4CRU"""
	def __init__(self):
		super(Robot4CRU, self).__init__()

		# Create subsciber for joint states published from arduino
		self.sub_joint_states = rospy.Subscriber("joint_state", JointState, self.callback_joint_state)

		# Create service for inverse kinematics
		self.robot_4cru_service = rospy.Service('robot_4cru', RobotIK, self.robot_4cru_ik)

		# Set up constant geometric parameters for the robot
		# self.set_geometric_params([1, 0, 0, 4]) # good schoenfiles workspace r =  85.8  mm(H_1 only)
		self.set_geometric_params([1, 0, 0, 1]) # Condition A

		# Intialize fixed robot tf
		self.robot_base_tfm = tf.TransformerROS(True, rospy.Duration(10.0))
		robot_base_tfstamp = TransformStamped()
		robot_base_tfstamp.header.frame_id = 'base_coordinate'
		robot_base_tfstamp.header.stamp = rospy.Time.now() 
		robot_base_tfstamp.child_frame_id = 'eeff_coordinate'
		robot_base_tfstamp.transform.rotation.w = 1.0 # set valid quaternion
		self.robot_base_tfm.setTransform(robot_base_tfstamp)

		# print self.robot_base_tfm.asMatrix('base_coordinate', robot_base_tfstamp.header)

		# Initialize end-effector home config
		# self.robot_current_tfm = pose_base_to_eeff
		# pose_home = TransformStamped()
		# pose_home.header.frame_id = 'eeff_coordinate'
		# pose_home.child_frame_id = 'eeff_coordinate'

		# Initialize current joint state
		self.current_joint_state = JointState()
		self.current_joint_state.header.stamp = rospy.Time.now()
		self.current_joint_state.name = ["motor_1", "motor_2", "motor_3", "motor_4"]
		self.current_joint_state.position = [100.0, 100.0, 100.0, 100.0]
		# print self.current_joint_state


		# Initialize pose in Schoenflies mode
		self.operation_mode = "H1"
		self.robot_pose_4dof = np.array([10, 10, 120.0, np.pi/12])

		pose_base_to_eeff, X, Y, Z, x_0, x_1, x_2, x_3 = self.convert_pose_4dof_to_tf_mat_and_7var(self.robot_pose_4dof)
		self.robot_pose = pose_base_to_eeff

		# test the ik function
		dummy_req = RobotIKRequest()
		dummy_pose = Pose()
		dummy_pose.position.x = X
		dummy_pose.position.y = Y
		dummy_pose.position.z = Z
		dummy_pose.orientation.x = x_1
		dummy_pose.orientation.y = x_2
		dummy_pose.orientation.z = x_3
		dummy_pose.orientation.w = x_0

		# dummy_req.des_poses.poses.append(dummy_pose)
		# dummy_req.des_poses.poses.append(dummy_pose)

		# alternative way but yields different results when printing!
		# dummy_req.des_poses.poses.append(Pose(Point(x = X, y = Y, z = Z), Quaternion(x= x_1, y= x_2, z = x_3, w = x_0)))

		# self.robot_4cru_ik(dummy_req)

	def set_geometric_params(self, new_geometric_indcs):
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
		self.r = (1.75*2 + 2.54*2)*10.0
		self.joint_pos_range = np.array([0, 200.00])
		self.joint_pos = np.full(4, self.joint_pos_range[1]/2.0) # home position
		self.h_offset = 3.0*25.4 # (approx.) TODO: update from real CAD

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

	def inverse_kinematics(self, pose_base_to_eeff):
		reals, discriminants, has_solution = self.check_ik_feasible(pose_base_to_eeff)
		all_joint_pos_sol = np.full((4, 2), self.joint_pos_range[1]/2.0) # home position as default value
		all_swivel_angles = np.zeros((4,2)) # indices [joint_no, U1/U2]
		current_joint_pos = np.array(self.current_joint_state.position)
		selected_joint_pos_sol = current_joint_pos

		if has_solution:
			for i in range(4):
				for j in range(2):
					if j == 0:
						all_joint_pos_sol[i,j] = reals[i] - np.sqrt(discriminants[i]) - self.h_offset
					elif j == 1:
						all_joint_pos_sol[i,j] = reals[i] + np.sqrt(discriminants[i]) - self.h_offset
					swivel_angle_U1, swivel_angle_U2 = self.calc_swivel_angle(pose_base_to_eeff, all_joint_pos_sol[i,1], i)
					all_swivel_angles[i,0] = swivel_angle_U1
					all_swivel_angles[i,1] = swivel_angle_U2
				joint_pos_diff_j_index = np.argmin(np.abs(all_joint_pos_sol[i,:] - current_joint_pos[i]))
				selected_joint_pos_sol[i] = all_joint_pos_sol[i, joint_pos_diff_j_index]
		else:
			# if no solution, return current joint positions
			print "No IK Solution: at least one discriminant is negative: ", discriminants

		# check the swivel angle limit on both ends of the U-U rod: provide warning on the screen if the joint are out of ranges
		print "IK joint pos solutions: ", selected_joint_pos_sol
		print "swivel angles (deg): ", np.rad2deg(all_swivel_angles)

		return selected_joint_pos_sol.tolist()

	def calc_swivel_angle(self, pose_base_to_eeff, joint_pos, joint_index):
		X, Y, Z, x_0, x_1, x_2, x_3 = get_7var_from_pose(pose_base_to_eeff)
		
		# convert signs based on joint indices
		if joint_index == 0:
			a = self.a; b = -self.b; c = self.c; d = -self.d
		elif joint_index == 1:
			a = self.a; b = self.b; c = self.c; d = self.d
		elif joint_index == 2:
			a = -self.a; b = self.b; c = -self.c; d = self.d
		elif joint_index == 3:
			a = -self.a; b = -self.b; c = -self.c; d = -self.d

		u0_B_to_C = np.array([X - a + c*(x_0**2 + x_1**2 - x_2**2 - x_3**2) + d*(-2.0*x_0*x_3 + 2.0*x_1*x_2),
			Y - b + c*(2.0*x_0*x_3 + 2.0*x_1*x_2) + d*(x_0**2 - x_1**2 + x_2**2 - x_3**2),
			Z + c*(-2.0*x_0*x_2 + 2.0*x_1*x_3) + d*(2.0*x_0*x_1 + 2.0*x_2*x_3) - joint_pos - self.h_offset])
		v0_C_to_ee = -np.array([c*(x_0**2 + x_1**2 - x_2**2 - x_3**2) + d*(-2.0*x_0*x_3 + 2.0*x_1*x_2),
			c*(2.0*x_0*x_3 + 2.0*x_1*x_2) + d*(x_0**2 - x_1**2 + x_2**2 - x_3**2),
			c*(-2.0*x_0*x_2 + 2.0*x_1*x_3) + d*(2.0*x_0*x_1 + 2.0*x_2*x_3)])
		v0_B_to_base = -np.array([a, b, 0])

		swivel_angle_U1 = py_ang(u0_B_to_C, v0_B_to_base)
		swivel_angle_U2 = py_ang(v0_C_to_ee, u0_B_to_C)

		return swivel_angle_U1, swivel_angle_U2

	def check_ik_feasible(self, pose_base_to_eeff):		
		X, Y, Z, x_0, x_1, x_2, x_3 = get_7var_from_pose(pose_base_to_eeff)
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

	def convert_pose_4dof_to_tf_mat_and_7var(self, pose_4dof):
		if self.operation_mode == "H1":
			t_mat = tfs.translation_matrix((pose_4dof[0], pose_4dof[1], pose_4dof[2]))
			r_mat = tfs.rotation_matrix(pose_4dof[3], (0, 0, 1))
			tf_mat_base_to_eeff = tfs.concatenate_matrices(t_mat, r_mat)
		if self.operation_mode == "H3":
			# break down to geometric cases
			pass
		else:
			pass

		X, Y, Z, x_0, x_1, x_2, x_3 = convert_tf_mat_to_7var(tf_mat_base_to_eeff)

		return tf_mat_base_to_eeff, X, Y, Z, x_0, x_1, x_2, x_3


	def robot_4cru_ik(self, req):
		# Get req as datatype PoseArray des_poses (http://docs.ros.org/melodic/api/geometry_msgs/html/msg/PoseArray.html)
		# Give resp as datatype JointTrajectory des_joint_positions (http://docs.ros.org/api/trajectory_msgs/html/msg/JointTrajectory.html)
		resp_robot_4cru_ik = RobotIKResponse()
		resp_robot_4cru_ik.des_joint_positions.joint_names = ["motor_1", "motor_2", "motor_3", "motor_4"]
		resp_robot_4cru_ik.des_joint_positions.header.stamp = rospy.Time.now() 
		resp_robot_4cru_ik.des_joint_positions.header.frame_id = 'des_joint_positions'

		for i in range(len(req.des_poses.poses)):
			# print req.des_poses.poses[i]
			traj_point = JointTrajectoryPoint()
			traj_point.positions = self.inverse_kinematics(req.des_poses.poses[i])
			resp_robot_4cru_ik.des_joint_positions.points.append(traj_point)

		return resp_robot_4cru_ik

	def callback_joint_state(self, msg):
		# Update the joint states sent from Arduino
		self.current_joint_state = msg


def convert_tf_mat_to_7var(tf_mat):
	quat = tfs.quaternion_from_matrix(tf_mat)
	xyz = tfs.translation_from_matrix(tf_mat)
	X = xyz[0]; Y = xyz[1]; Z = xyz[2] 
	x_0 = quat[3]; x_1 = quat[0]; x_2 = quat[1]; x_3 = quat[2]
	return X, Y, Z, x_0, x_1, x_2, x_3

def py_ang(v1, v2):
		""" Returns the angle in radians between vectors 'v1' and 'v2'    """
		cosang = np.dot(v1, v2)
		sinang = la.norm(np.cross(v1, v2))
		return np.arctan2(sinang, cosang)

def convert_7var_to_pose(X, Y, Z, x_0, x_1, x_2, x_3):
	dummy_pose = Pose()
	dummy_pose.position.x = X
	dummy_pose.position.y = Y
	dummy_pose.position.z = Z
	dummy_pose.orientation.x = x_1
	dummy_pose.orientation.y = x_2
	dummy_pose.orientation.z = x_3
	dummy_pose.orientation.w = x_0
	return dummy_pose

def get_7var_from_pose(pose):
	X = pose.position.x
	Y = pose.position.y
	Z = pose.position.z
	x_1 = pose.orientation.x
	x_2 = pose.orientation.y
	x_3 = pose.orientation.z
	x_0 = pose.orientation.w
	return X, Y, Z, x_0, x_1, x_2, x_3

def main():
	rospy.init_node('robot_4cru', anonymous=True)
	robot = Robot4CRU()
	print "Spinning robot_4cru node ..."
	rospy.spin()

if __name__ == '__main__':
	main()
		
		
