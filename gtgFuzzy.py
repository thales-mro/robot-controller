import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from numpy.linalg import norm
import time





#def main():

class gtgFuzzyController():

	def __init__(self, robot):

		self.robot = robot
		

		deltaR = ctrl.Antecedent(np.arange(0, 50, 1), 'distance')
		deltaA = ctrl.Antecedent(np.arange(0, 8, 1), 'angle variation')
		velocity = ctrl.Consequent(np.arange(0, 6, 1), 'velocity')

		deltaR['far'] = fuzz.trapmf(deltaR.universe, [3, 4, 50, 50])
		deltaR['medium'] = fuzz.trapmf(deltaR.universe, [0.75, 2, 3, 4])
		deltaR['close'] = fuzz.trapmf(deltaR.universe, [0, 0, 0.5, 1.0])

		deltaA['high'] = fuzz.trapmf(deltaA.universe, [np.pi/2, 7*np.pi/12, 7.0, 7.0])
		deltaA['medium'] = fuzz.trapmf(deltaA.universe, [np.pi/6, np.pi/4, 5*np.pi/12, 7*np.pi/12])
		deltaA['low'] = fuzz.trapmf(deltaA.universe, [0, 0, np.pi/6, np.pi/4])

		velocity['fast'] = fuzz.trapmf(velocity.universe, [3.5, 5, 5, 5])
		velocity['normal'] = fuzz.trapmf(velocity.universe, [1.5, 2.5, 3.5, 4])
		velocity['slow'] = fuzz.trapmf(velocity.universe, [0, 0, 1.5, 2])


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
		
		stored_angles = self.getStoredAngles()
		inputToSystem = np.array([5.0]*len(stored_angles))

		thetas_mod = np.arccos(np.divide(vecs[:,0], norm(vecs, axis=1)))
		thetas = np.where(vecs[:,1] < 0, -1*thetas_mod, thetas_mod)

		for index in range(len(vecs)):
			diff = abs(thetas[index] - stored_angles)
			index_of_angle = np.argmin(diff)
			inputToSystem[index_of_angle] = norm(vecs[index])

		return inputToSystem, stored_angles


	def run(self, finish):

		self.finish = finish
		half = 684//2
		half_section = 21//2

		while(self.robot.get_connection_status() != -1):

			ir_distances = self.robot.read_laser()
			ir_distances = np.array(ir_distances).reshape(len(ir_distances)//3,3)[:684,:2]

			r, theta = self.getInputValues(ir_distances)

			detection_range = r[half-60:half+60]
			min_dist = np.min(detection_range)

			robot_pos = np.array(self.robot.get_current_position())[:2]

			dx, dy = self.finish - robot_pos
			errorDistance = np.sqrt((dx**2 + dy**2)) 

			print(min_dist, errorDistance)
			if min_dist >= 1.0 or errorDistance <= 1.0:
				robot_angles = self.robot.get_current_orientation()
				theta = robot_angles[2]

				

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
				

				if abs(errorAngle) <= 0.1:
					self.robot.set_left_velocity(vel)
					self.robot.set_right_velocity(vel)

				elif errorDistance <= 0.1:
					self.robot.set_left_velocity(0.0)
					self.robot.set_right_velocity(0.0)
					return 1

				elif errorAngle < 0:
					self.robot.set_left_velocity(0.0)
					self.robot.set_right_velocity(vel)

				elif errorAngle > 0:
					self.robot.set_left_velocity(vel)
					self.robot.set_right_velocity(0.0)

				time.sleep(0.1)

			else:
				return 2



	




#if __name__ == '__main__':
#	main()