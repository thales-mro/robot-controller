import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from numpy.linalg import norm
import time


class gtgFuzzyController():

	def __init__(self, robot, odometry):

		self.robot = robot
		self.odometry = odometry
		
		'''
		Definition of the Fuzzy System to GoToGoal. We define as consequent 
		'deltaR' as the distance to the goal and 'deltaA' as the angle of the
		robot related to the straight line that joins the final target and 
		currently position of teh robot

		'''
		deltaR = ctrl.Antecedent(np.arange(0, 50, 0.1), 'distance')
		deltaA = ctrl.Antecedent(np.arange(0, 8, 0.1), 'angle variation')
		velocity = ctrl.Consequent(np.arange(0, 6, 0.1), 'velocity')

		deltaR['far'] = fuzz.trapmf(deltaR.universe, [3, 4, 50, 50])
		deltaR['medium'] = fuzz.trapmf(deltaR.universe, [0.75, 2, 3, 4])
		deltaR['close'] = fuzz.trapmf(deltaR.universe, [0, 0, 0.5, 1.0])

		deltaA['high'] = fuzz.trapmf(deltaA.universe, [np.pi/2, 7*np.pi/12, 7.0, 7.0])
		deltaA['medium'] = fuzz.trapmf(deltaA.universe, [np.pi/6, np.pi/4, 5*np.pi/12, 7*np.pi/12])
		deltaA['low'] = fuzz.trapmf(deltaA.universe, [0, 0, np.pi/6, np.pi/4])

		velocity['fast'] = fuzz.trapmf(velocity.universe, [2.5, 3.0, 5.0, 5.0])
		velocity['normal'] = fuzz.trapmf(velocity.universe, [1.0, 1.25, 2.5, 3.0])
		velocity['slow'] = fuzz.trapmf(velocity.universe, [0, 0, 1.0, 1.25])


		# Definiton of rueles
		rule1 = ctrl.Rule(deltaR['far'] & (deltaA['high'] | deltaA['medium']), velocity['normal'])
		rule2 = ctrl.Rule(deltaR['far'] & deltaA['low'], velocity['fast'])

		rule3 = ctrl.Rule(deltaR['medium'] & deltaA['high'], velocity['normal'])
		rule4 = ctrl.Rule(deltaR['medium'] & (deltaA['medium'] | deltaA['low']), velocity['normal'])

		rule5 = ctrl.Rule(deltaR['close'] & deltaA['high'], velocity['normal'])
		rule6 = ctrl.Rule(deltaR['close'] & (deltaA['medium'] | deltaA['low']), velocity['slow'])
		

		gtg_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6])


		self.gtg = ctrl.ControlSystemSimulation(gtg_ctrl)

	def computeVelocity(self, ErrorDistance, ErrorAngle):
		self.gtg.input['distance'] = ErrorDistance
		self.gtg.input['angle variation'] = ErrorAngle

		self.gtg.compute()

		vel = self.gtg.output['velocity']
		return vel

	def getStoredAngles(self):
		return np.load("stored_angles.npy")

	def getInputValues(self, vecs):
		'''
        It returns the distance that each beacon of laser sensor captured.
        Since neither all the beacons returns a position of an obstacle 
        (beacons that did not detect anything did not return any position)
        we need to set as 5.0 to the beacon that did not detect anything and
        the distance of obstacle to the beacons that detect it. So we cross 
        the information between the position detected by a beacon and its angle
        and then we can assign or the distance of the object (if detected) or 5.0
        (if not detected)

        Keyword:
        vecs -- vector of the positions of obstacles detected by the beacons

        '''
		stored_angles = self.getStoredAngles()
		inputToSystem = np.array([5.0]*len(stored_angles))

		thetas_mod = np.arccos(np.divide(vecs[:,0], norm(vecs, axis=1)))
		thetas = np.where(vecs[:,1] < 0, -1*thetas_mod, thetas_mod)

		for index in range(len(vecs)):
			diff = abs(thetas[index] - stored_angles)
			index_of_angle = np.argmin(diff)
			inputToSystem[index_of_angle] = norm(vecs[index])

		return inputToSystem, stored_angles


	def run(self, finish, final_target, trajectory):

		self.finish = finish
		half = 684//2
		half_section = 21//2



		while(self.robot.get_connection_status() != -1):
			
			self.odometry.calculate_odometry()
			
			# Get sensor reading
			ir_distances = self.robot.read_laser()
			ir_distances = np.array(ir_distances).reshape(len(ir_distances)//3,3)[:684,:2]

			# Get distances for all of the 684 associated beacons
			r, theta = self.getInputValues(ir_distances)
			
			pos = np.array(self.robot.get_current_position())[:2]
			
			trajectory.append(pos)
			# Range of the beacons used to detect if there is something in front of the robot
			detection_range = r[half-55:half+55]
			min_dist = np.min(detection_range)

			robot_pos = self.odometry.get_pose()[:2]

			dx, dy = self.finish - robot_pos
			errorDistance = np.sqrt((dx**2 + dy**2)) 
			#print(errorDistance, self.finish)
			# If there is nothing in front of the robot, it calculates the velocity based on fuzzy system
			# The second argument of the "or" is: if the distance to the target is closer than the obstacle
			# then the robot keeps going until reach the target.
			if min_dist >= 0.80 or (errorDistance <= 0.80 and final_target):
				theta = self.odometry.get_pose()[2]

				dx, dy = self.finish - robot_pos
				alpha = np.arctan(dy/dx)

				if self.finish[0] >= robot_pos[0] and self.finish[1] <= robot_pos[1]:
					errorAngle = theta - alpha

				elif self.finish[0] <= robot_pos[0] and self.finish[1] >= robot_pos[1]:
					errorAngle = theta - (np.pi + alpha)

				elif self.finish[0] <= robot_pos[0] and self.finish[1] <= robot_pos[1]:
					errorAngle = theta - alpha + np.pi

				elif self.finish[0] >= robot_pos[0] and self.finish[1] >= robot_pos[1]:
					errorAngle = theta - alpha


				errorDistance = np.sqrt((dx**2 + dy**2)) 

				#print(errorAngle, errorDistance)
				vel = self.computeVelocity(errorDistance, errorAngle)
				
				# If the error angle is very low, then the vlelocity in both wheels are the same.
				if abs(errorAngle) <= 0.1:
					self.robot.set_left_velocity(vel)
					self.robot.set_right_velocity(vel)

				elif errorDistance <= 0.1:
					self.robot.set_left_velocity(0.0)
					self.robot.set_right_velocity(0.0)
					return 1, trajectory

				# Adjust the velocity of a wheel based on the error angle. 
				elif errorAngle < 0:
					self.robot.set_left_velocity(0.0)
					self.robot.set_right_velocity(vel)

				elif errorAngle > 0:
					self.robot.set_left_velocity(vel)
					self.robot.set_right_velocity(0.0)

				time.sleep(0.1)

			else:
				return 2, trajectory



	




#if __name__ == '__main__':
#	main()