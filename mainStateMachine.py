import sys
sys.path.insert(0, 'src')
sys.path.insert(0, 'lib')
from robot import Robot
import numpy as np
from numpy.linalg import norm
import time

from avoidObstacleFuzzy02 import avoidObstacleFuzzyController
from gtgFuzzy import gtgFuzzyController
from wallFollow import WallFollowController
from odometry import Odometry

import argparse
#### ---- Mapping of states ---- ####
'''
0: Waiting for input -- dummy one
1: GotoGo
2: Avoid Obstacle
3: Wall Follow
4: stop
'''
#####################################
def main():

	parser = argparse.ArgumentParser()
	arg = parser.add_argument
	arg('--mode', required=True)
	args = parser.parse_args()

	mode = int(args.mode)

	robot = Robot() # Instantiates a robot that will be used along all the algorithm
	finishes = np.array([[3.0, 2.5]]) # Define a finish point
	#finishes = np.array([[2.05, -1.175]]) 
	x, y = robot.get_current_position()[0:2]
	orientation = robot.get_current_orientation()[2]
	
	odom = Odometry(robot, x, y, orientation, mode=mode)
	trajectory = []
	# ----- Instantiates each beahavior separately ---- #
	avoidObsCtrl = avoidObstacleFuzzyController(robot, odom)
	gtgCtrl = gtgFuzzyController(robot, odom)
	wallFollowCtrl = WallFollowController(robot, odom)
	# ------------------------------------------------- #

	'''
	So far, the state machine has 4 states:
		state 0: the initial state. It is basically a start in GoToGoal.

		state 1: GoToGoal. Before being executed, the finishes list is verified.
				 The finishes list holds all the finish points, since after AvoidObstacle
				 being performed, a new provisory goal is set to be achieved by the robot
				 in order to avoid a collision.

		state 2: Avoid Obstacle. After being executed, it adds a new goal in the finishes list.
				 Within this code, we have a threshold to check how much the robot turned in order
				 to see if the robot is avoiding an obstacle or a wall. If it is avoiding an obstacle
				 then the WallFollow state is returned and the parameters of the side and sensor are
				 set in WallFollow state.

		state 3: WallFollow. It performs the WallFollow given the paramters obtained in state 2.

		state 4: Final State. It is achieved after the robot arrived in the end point set iniatilly.
	'''
	state = 0
	while state != 4:
		# this first 'if' is only used to map the first state
		# it's redundat and a dummy one, only used as start point
		if state == 0:
			state, trajectory = gtgCtrl.run(finishes[-1], True, trajectory)

			if state == 1:
				finishes = np.delete(finishes, -1, axis=0)

			#robot.stop()
			#time.sleep(0.1)

		elif state == 1:
			if len(finishes) > 0:

				if len(finishes) == 1:
					state, trajectory = gtgCtrl.run(finishes[-1], True, trajectory)

					if state == 1:
						finishes = np.delete(finishes, -1, axis=0)

				else:
					state, trajectory = gtgCtrl.run(finishes[-1], False, trajectory)
					finishes = np.delete(finishes, -1, axis=0)

			else:
				state = 4

			#robot.stop()
			#time.sleep(0.1)


		elif state == 2:
			state, trajectory = avoidObsCtrl.run(trajectory)

			if state == 1:
				new_finish = avoidObsCtrl.getDefinedFinish()
				finishes = np.append(finishes, new_finish, axis=0)
			
			#robot.stop()
			#time.sleep(0.1)

		elif state == 3:
			sensor_number = avoidObsCtrl.sensor_number
			sensor_side = avoidObsCtrl.sensor_side
			state, trajectory = wallFollowCtrl.run(sensor_number, sensor_side, trajectory)
			
			#robot.stop()
			#time.sleep(0.1)


		elif state == 4:
			robot.stop()
			break

		#print(finishes)

	robot.stop()
	#print(robot.get_current_position(), odom.get_pose())
	np.save("trajectory_performed_%d.npy" % mode, trajectory, fix_imports=False)
	print("Done!")


if __name__ == '__main__':
	main()