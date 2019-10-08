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
	mode = 0

	if(robot.get_connection_status() != -1):
		pos_ref = np.array(robot.get_current_position())[:2]

	while(robot.get_connection_status() != -1):


		ir_distances = robot.read_laser()
		ir_distances = np.array(ir_distances).reshape(len(ir_distances)//3,3)[:684,:2]

		
		r, theta = avoidObsCtrl.getInputValues(ir_distances)

		'''
		if mode == 0:
			detection_range = r[half-100:half+100]
			min_dist = np.min(detection_range)

			if min_dist <= 0.5:
				mode = 1
				pos0 = np.array(robot.get_current_position())[:2]
				vecRef = pos0 - pos_ref

				robot.set_left_velocity(0.0)
				robot.set_right_velocity(2.0)

			else:
				robot.set_left_velocity(2.0)
				robot.set_right_velocity(2.0)
		'''
		#elif mode == 1:

		theta = theta[::11]
		r = r[::11]
		vel_left, vel_right = avoidObsCtrl.getVelocities(r)
		
		robot.set_left_velocity(vel_left)
		robot.set_right_velocity(vel_right)

		time.sleep(0.1)

		'''
		pos1 = np.array(robot.get_current_position())[:2]
		vecActual = pos1 - pos0

		theta = np.arccos(np.sum(vecRef*vecActual)/(norm(vecRef)*norm(vecActual)))

		if theta < 0.01:
			mode = 0
			robot.set_left_velocity(0.0)
			robot.set_right_velocity(2.0)

		
		print(mode)
		'''

		#for i in range(len(theta)-1):
		#	print((180*(theta[i] - theta[i+1]))/np.pi)

		#x = r*np.cos(theta)
		#y = r*np.sin(theta)
		

		#red = np.where(r == 5.0)[0]
		#blue = np.where(r < 5.0)[0]

		#sec = len(r)//3
		#plt.plot(-1*x[:sec], -1*y[:sec], 'r.')
		#plt.plot(-1*x[sec:2*sec], -1*y[sec:2*sec], 'b.')
		#plt.plot(-1*x[2*sec:], -1*y[2*sec:], 'g.')
		#plt.show()
	

		
		

if __name__ == '__main__':
	main()
'''
# BACKUP
half = len(ir_distances)//2

		ir_frontal = ir_distances[half-100:half+100]

		threshold = 0.3

		index_of_lowest = np.argmin(ir_frontal)

		if norm(ir_frontal[index_of_lowest]) <= threshold:
			ir_right = ir_distances[half+100:]
			ir_left = ir_distances[:half-100]

			index_of_lowest_right = avoidObsCtrl.getMinimumLast(ir_right)
			index_of_lowest_left = avoidObsCtrl.getMinimumLast(ir_left)

			if norm(ir_right[index_of_lowest_right]) >= norm(ir_left[index_of_lowest_left]):
				vec_frontal = ir_distances[half]
				vec_right = ir_right[index_of_lowest_right]

				dtheta = np.arccos((vec_frontal*vec_right)/(norm(vec_frontal)*norm(vec_right)))
				theta0 = robot.get_current_orientation()
'''

'''
		stored_angles = []
		for i in range(len(ir_distances)):
			theta = np.arccos((np.sum(ir_distances[i]*ds))/(norm(ir_distances[i])*norm(ds)))
			signal = ir_distances[i][1]/abs(ir_distances[i][1])
			stored_angles.append(signal*theta)

		np.save("stored_angles", stored_angles, fix_imports=False)
		'''