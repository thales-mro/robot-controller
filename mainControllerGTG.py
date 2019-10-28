import skfuzzy
import sys
sys.path.insert(0, 'src')
sys.path.insert(0, 'lib')
from robot import Robot
import numpy as np
from numpy.linalg import norm
import time

from gtgFuzzy import gtgFuzzyController

from odometry import Odometry
import argparse


def main():


	robot = Robot() # Instantiates a robot that will be used along all the algorithm
	finish = np.array([3.0, 2.5]) # Define a finish point
	
	parser = argparse.ArgumentParser()
	arg = parser.add_argument
	arg('--mode', required=True)
	args = parser.parse_args()

	mode = int(args.mode)
	
	x, y = robot.get_current_position()[0:2]
	orientation = robot.get_current_orientation()[2]
	mode = 2
	odom = Odometry(robot, x, y, orientation, mode=mode)
	trajectory = []
	final_target = True
	gtgCtrl = gtgFuzzyController(robot, odom)
	

	half = 684//2
	half_section = 21//2



	while(robot.get_connection_status() != -1):
		
		odom.calculate_odometry()
		
		# Get sensor reading
		ir_distances = robot.read_laser()
		ir_distances = np.array(ir_distances).reshape(len(ir_distances)//3,3)[:684,:2]

		# Get distances for all of the 684 associated beacons
		r, theta = gtgCtrl.getInputValues(ir_distances)
		
		pos = np.array(robot.get_current_position())[:2]
		
		trajectory.append(pos)
		# Range of the beacons used to detect if there is something in front of the robot
		detection_range = r[half-55:half+55]
		min_dist = np.min(detection_range)

		robot_pos = odom.get_pose()[:2]

		dx, dy = finish - robot_pos
		errorDistance = np.sqrt((dx**2 + dy**2)) 
		#print(errorDistance, self.finish)
		# If there is nothing in front of the robot, it calculates the velocity based on fuzzy system
		# The second argument of the "or" is: if the distance to the target is closer than the obstacle
		# then the robot keeps going until reach the target.
		print(errorDistance)
		if min_dist >= 0.80 or (errorDistance <= 0.80 and final_target):
			theta = odom.get_pose()[2]

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

			#print(errorAngle, errorDistance)
			vel = gtgCtrl.computeVelocity(errorDistance, errorAngle)
			
			# If the error angle is very low, then the vlelocity in both wheels are the same.
			if abs(errorAngle) <= 0.1:
				robot.set_left_velocity(vel)
				robot.set_right_velocity(vel)

			elif errorDistance <= 0.1:
				robot.set_left_velocity(0.0)
				robot.set_right_velocity(0.0)
				time.sleep(0.1)
				break

			# Adjust the velocity of a wheel based on the error angle. 
			elif errorAngle < 0:
				robot.set_left_velocity(0.0)
				robot.set_right_velocity(vel)

			elif errorAngle > 0:
				robot.set_left_velocity(vel)
				robot.set_right_velocity(0.0)

			time.sleep(0.1)

	np.save("trajectory_performed_GoToGoal_%d.npy" % mode, trajectory, fix_imports=False)
		

if __name__ == '__main__':
	main()