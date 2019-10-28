import skfuzzy
import sys
sys.path.insert(0, 'src')
sys.path.insert(0, 'lib')
from robot import Robot
import numpy as np
from numpy.linalg import norm
import time

from avoidObstacleFuzzy02 import avoidObstacleFuzzyController
import matplotlib.pyplot as plt

from odometry import Odometry
import argparse

def main():

	parser = argparse.ArgumentParser()
	arg = parser.add_argument
	arg('--mode', required=True)
	args = parser.parse_args()

	mode = int(args.mode)
	
	robot = Robot()

	x, y = robot.get_current_position()[0:2]
	orientation = robot.get_current_orientation()[2]
	odom = Odometry(robot, x, y, orientation, mode=mode)
	
	avoidObsCtrl = avoidObstacleFuzzyController(robot, odom)
	
	half = 684//2
	half_section = 21//2
	threshold = np.pi/3 

	trajectory = []
	
	if(robot.get_connection_status() != -1):
		angle_ref = odom.get_pose()[-1]

	while(robot.get_connection_status() != -1):
		#self.odometry.calculate_odometry()
		# Get sensor reading
		ir_distances = robot.read_laser()
		ir_distances = np.array(ir_distances).reshape(len(ir_distances)//3,3)[:684,:2]

		# Get distances for all of the 684 associated beacons
		r, theta = avoidObsCtrl.getInputValues(ir_distances)
		

		pos = np.array(robot.get_current_position())[:2]
		
		trajectory.append(pos)

		# Range of the beacons used to detect if there is something in front of the robot
		detection_range = r[half-55:half+55]
		min_dist = np.min(detection_range)
		
		if min_dist <= 0.80: # Verify if the robot is free to go forward
			r1 = r[:half]	
			r_downsampled = r1

			velLeft, velRight = avoidObsCtrl.getVelocities(r_downsampled)

			#print(velLeft, velRight)
			robot.set_left_velocity(velLeft)
			robot.set_right_velocity(velRight)
			#time.sleep(0.1)

		else: # After change the orientation, a new provisory goal is set.
			angle_final = odom.get_pose()[-1]
			actual_pos = odom.get_pose()[:2]

			r = 1.5

			# Since we do not previous know if the robot is avoiding a obstacle or a wall
			# We set the sensor side and the sensor number if the former option happens.
			if angle_ref <= -np.pi/2 and angle_final >= np.pi/2: 
				dtheta = angle_ref - angle_final + 2*np.pi
				sensor_side = "left"
				sensor_number = 15

			elif angle_final <= -np.pi/2 and angle_ref >= np.pi/2: 
				dtheta = angle_final - angle_ref + 2*np.pi
				sensor_side = "right"
				sensor_number = 8

			else:
				dtheta = abs(angle_final - angle_ref)

				if angle_final > angle_ref:
					sensor_side = "right"
					sensor_number = 7
				else:
					sensor_side = "left"
					sensor_number = 0

			break			

	robot.set_left_velocity(3.0)
	robot.set_right_velocity(3.0)
	time.sleep(0.5)
	robot.stop()
	np.save("trajectory_performed_AvoidObstacle_%d.npy" % mode, trajectory, fix_imports=False)
	
if __name__ == '__main__':
	main()
