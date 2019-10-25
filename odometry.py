import sys
sys.path.insert(0, 'src')
sys.path.insert(0, 'lib')
from robot import Robot
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import time
import vrep

# Select the Mode, Circuit and Filtering
# Mode can be 1, 2 or 3 (encoders, gyroscope or fusion)
MODE = 2#int(args.mode)
# 0 or 1. A boolean value. It is used to decide if we multiply or not the angle by a factor.
FILTER = 0#bool(args.filtering)

class Odometry:
	"""
	Odometry calculation class.
	It takes into consideration the odometry mode and the previous pose, if needed
	"""

	def __init__(self, robot, x=0, y=0, orientation=0, mode=0):
		self.robot = robot
		self.x = x
		self.y = y
		self.orientation = orientation
		self.mode = mode
		self.old_gyro_read = 0
		# triggers streaming mode
		res, gyroZ = vrep.simxGetFloatSignal(self.robot.clientID, "gyroZ", vrep.simx_opmode_streaming)


	def get_pose(self):
		"""
		It returns the pose of robot in the tuple format (x, y, theta)
		"""
		return (self.x, self.y, self.orientation)

	def calculate_odometry(self):
		"""
		It calculates the odometry in this execution step
		"""
		if self.mode == 0:
			self.x, self.y = self.robot.get_current_position()[0:2]
			self.orientation = self.robot.get_current_orientation()[2]
		else:
			# read initial angles from each wheel
			angle0_left = self._get_left_encoder(self.robot)
			angle0_right = self._get_right_encoder(self.robot)

			# read theta variation from gyroscope
			valid = False
			while(not valid):
				res, gyroZ = vrep.simxGetFloatSignal(self.robot.clientID, "gyroZ", vrep.simx_opmode_buffer)
				if res == 0 and gyroZ != self.old_gyro_read:
					self.old_gyro_read = gyroZ
					valid = True
			#print("gyro results:", res, gyroZ)

			# read final angles from each wheel
			angle1_left = self._get_left_encoder(self.robot)
			angle1_right = self._get_right_encoder(self.robot)


			# Deal with the discontinuity of the angles
			if angle0_left > 0.0 and angle1_left < 0.0:
				dtheta_left = (angle1_left + np.pi) + (np.pi - angle0_left)
			else:
				dtheta_left = abs(angle1_left - angle0_left)

			if angle0_right > 0.0 and angle1_right < 0.0:
				dtheta_right = (angle1_right + np.pi) + (np.pi - angle0_right)
			else:
				dtheta_right = abs(angle1_right - angle0_right)


			# Performs the comparison statment to decide which mode and Filter state the
			# robot needs to follow to get Delta Theta
			if MODE == 1:
				dangle_encoder = ((self.robot.WHEEL_RADIUS*(dtheta_right - dtheta_left))/self.robot.ROBOT_WIDTH)

				if FILTER:
					dangle = dangle_encoder*1.05
				else:
					dangle = dangle_encoder

			elif MODE == 2:

				if FILTER:
					dangle = gyroZ*0.85
				else:
					dangle = gyroZ


			elif MODE == 3:
				dangle_encoder = (self.robot.WHEEL_RADIUS*(dtheta_right - dtheta_left))/self.robot.ROBOT_WIDTH #-- encoder odometry

				if FILTER:
					dangle = (0.9*gyroZ + 1.05*dangle_encoder)/2
				else:
					dangle = (gyroZ + dangle_encoder)/2 

			# Get the x and y variations
			ds = (self.robot.WHEEL_RADIUS*(dtheta_right + dtheta_left))/2

			dx = ds*np.cos(self.orientation + dangle/2)
			dy = ds*np.sin(self.orientation + dangle/2)

			self.x += dx
			self.y += dy
			self.orientation += dangle

	# The two folloing fucntions are used to get the reading from the angle detetcion of the wheels. We treat them
	# as encoders since they behave very similar to one enconder.
	def _get_left_encoder(self, robot):
		values = vrep.simxGetJointPosition(robot.clientID, robot.motors_handle["left"], vrep.simx_opmode_streaming)
		return values[1]

	def _get_right_encoder(self, robot):
		values = vrep.simxGetJointPosition(robot.clientID, robot.motors_handle["right"], vrep.simx_opmode_streaming)
		return values[1]

