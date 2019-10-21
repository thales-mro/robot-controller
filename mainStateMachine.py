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

	robot = Robot()
	finishes = np.array([[3.0, 2.5]])
	avoidObsCtrl = avoidObstacleFuzzyController(robot)
	gtgCtrl = gtgFuzzyController(robot)
	wallFollowCtrl = WallFollowController(robot)


	state = 0
	while state != 4:
		#print(state)
		# this first 'if' is only used to map the first state
		# it's redundat and a dummy one, only used as start point
		if state == 0:
			state = gtgCtrl.run(finishes[-1], True)

			if state == 1:
				finishes = np.delete(finishes, -1, axis=0)

			print(state, finishes)

		elif state == 1:
			if len(finishes) > 0:

				if len(finishes) == 1:
					state = gtgCtrl.run(finishes[-1], True)

					if state == 1:
						finishes = np.delete(finishes, -1, axis=0)

				else:
					state = gtgCtrl.run(finishes[-1], False)
					finishes = np.delete(finishes, -1, axis=0)

			else:
				state = 4

			print(state, finishes)

		elif state == 2:
			state = avoidObsCtrl.run()

			if state == 1:
				new_finish = avoidObsCtrl.getDefinedFinish()
				finishes = np.append(finishes, new_finish, axis=0)
			
			robot.stop()
			time.sleep(0.2)
			print(state, finishes)


		elif state == 3:
			sensor_number = avoidObsCtrl.sensor_number
			sensor_side = avoidObsCtrl.sensor_side
			state = wallFollowCtrl.run(sensor_number, sensor_side)
			print(state, finishes)
			robot.stop()
			time.sleep(0.2)

		elif state == 4:
			robot.stop()
			break

	robot.stop()
	print("Done!")


if __name__ == '__main__':
	main()