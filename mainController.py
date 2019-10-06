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

	finish = np.array([3.2, -1.5])

	while(robot.get_connection_status() != -1):

		#robot.set_left_velocity(3.0)
		#robot.set_right_velocity(3.0)

		robot_angles = robot.get_current_orientation()
		theta = robot_angles[2]

		robot_pos = np.array(robot.get_current_position())[:2]

		dx, dy = finish - robot_pos
		alpha = np.arctan(dy/dx)

		errorAngle = theta - alpha
		errorDistance = np.sqrt((dx**2 + dy**2)) 

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