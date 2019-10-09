import skfuzzy
import sys
sys.path.insert(0, 'src')
sys.path.insert(0, 'lib')
from robot import Robot
import numpy as np
from numpy.linalg import norm
import time

from avoidObstacleFuzzy import avoidObstacleFuzzyController
import matplotlib.pyplot as plt


def main():

	

	robot = Robot()
	avoidObsCtrl = avoidObstacleFuzzyController(robot)
	half = 684//2
	half_section = 21//2
	mode = 0

	if(robot.get_connection_status() != -1):
		pos_ref = np.array(robot.get_current_position())[:2]

	while(robot.get_connection_status() != -1):


		ir_distances = robot.read_laser()
		ir_distances = np.array(ir_distances).reshape(len(ir_distances)//3,3)[:684,:2]

		r, theta = avoidObsCtrl.getInputValues(ir_distances)

		detection_range = r[half-35:half+35]
		min_dist = np.min(detection_range)

		r1 = r[::11][:21][half_section-3:half_section+3]
		r2 = r[::11][21:42][half_section-3:half_section+3]
		r3 = r[::11][42:][half_section-3:half_section+3]

		r_downsampled = np.concatenate((r1, r2, r3))

		if mode == 0:
			if min_dist <= 1.0:
				mode = 1
				velLeft, velRight = avoidObsCtrl.getVelocities(r_downsampled)
				print(velLeft, velRight)
				robot.set_left_velocity(velLeft)
				robot.set_right_velocity(velRight)

			else:

				robot.set_left_velocity(2.0)
				robot.set_right_velocity(2.0)


		elif mode == 1:
			if min_dist >= 1.0:
				mode = 0
				robot.set_left_velocity(2.0)
				robot.set_right_velocity(2.0)

			else:
				velLeft, velRight = avoidObsCtrl.getVelocities(r_downsampled)
				print(velLeft, velRight)
				robot.set_left_velocity(velLeft)
				robot.set_right_velocity(velRight)
		
if __name__ == '__main__':
	main()
