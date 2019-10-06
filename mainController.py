import skfuzzy
import sys
sys.path.insert(0, 'src')
sys.path.insert(0, 'lib')
from robot import Robot
import numpy as np
from numpy.linalg import norm
import time

from gtgFuzzy import gtgFuzzyController



def main():


	robot = Robot()
	gtgCtrl = gtgFuzzyController()

	#finish = np.array([3.2, -1.5])
	#finish = np.array([0.0, 2.0]) 
	#finish = np.array([0.0, 0.0])
	finish = np.array([3.0, 2.0])

	while(robot.get_connection_status() != -1):

		#robot.set_left_velocity(3.0)
		#robot.set_right_velocity(3.0)

		robot_angles = robot.get_current_orientation()
		theta = robot_angles[2]

		robot_pos = np.array(robot.get_current_position())[:2]

		dx, dy = finish - robot_pos
		alpha = np.arctan(dy/dx)

		if finish[0] >= robot_pos[0] and finish[1] <= robot_pos[1]:
			errorAngle = theta - alpha

		elif finish[0] <= robot_pos[0] and finish[1] >= robot_pos[1]:
			errorAngle = theta - (np.pi + alpha)

		elif finish[0] <= robot_pos[0] and finish[1] <= robot_pos[1]:
			errorAngle = theta - alpha + np.pi

		elif finish[0] >= robot_pos[0] and finish[1] >= robot_pos[1]:
			errorAngle = theta - alpha


		errorDistance = np.sqrt((dx**2 + dy**2)) 

		print(errorAngle, errorDistance)
		vel = gtgCtrl.computeVelocity(errorDistance, errorAngle)
		

		if abs(errorAngle) <= 0.1:
			robot.set_left_velocity(vel)
			robot.set_right_velocity(vel)

		elif errorDistance <= 0.1:
			robot.set_left_velocity(0.0)
			robot.set_right_velocity(0.0)
			break

		elif errorAngle < 0:
			robot.set_left_velocity(0.0)
			robot.set_right_velocity(vel)

		elif errorAngle > 0:
			robot.set_left_velocity(vel)
			robot.set_right_velocity(0.0)

		time.sleep(0.1)













if __name__ == '__main__':
	main()