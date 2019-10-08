import numpy as np
from numpy.linalg import norm
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt


class avoidObstacleFuzzyController():


	def __init__(self, robot):
		self.robot = robot

		
		velocityRight = ctrl.Consequent(np.arange(0, 5, 0.1), 'velocity right')
		velocityLeft = ctrl.Consequent(np.arange(0, 5, 0.1), 'velocity left')

		velocityRight['very slow'] = fuzz.trimf(velocityRight.universe, [0.0, 0.0, 0.5])
		velocityRight['slow'] = fuzz.trimf(velocityRight.universe, [0.5, 1.0, 1.5])
		velocityRight['medium'] = fuzz.trimf(velocityRight.universe, [1.5, 2.0, 2.5])
		velocityRight['fast'] = fuzz.trimf(velocityRight.universe, [2.5, 3.0, 3.5])
		velocityRight['very fast'] = fuzz.trimf(velocityRight.universe, [3.5, 4.0, 4.5])

		#x = fuzz.trimf(velocityRight.universe, [0.4, 0.7, 1.0]) 
		#plt.plot(velocityRight.universe, x, 'b.')
		#plt.show()

		velocityLeft['very slow'] = fuzz.trimf(velocityLeft.universe, [0.0, 0.0, 0.5])
		velocityLeft['slow'] = fuzz.trimf(velocityLeft.universe, [0.5, 1.0, 1.5])
		velocityLeft['medium'] = fuzz.trimf(velocityLeft.universe, [1.5, 2.0, 2.5])
		velocityLeft['fast'] = fuzz.trimf(velocityLeft.universe, [2.5, 3.0, 3.5])
		velocityLeft['very fast'] = fuzz.trimf(velocityLeft.universe, [3.5, 4.0, 4.5])

		# East Sector
		section_split = 63//3

		rulesSectorEast = [[],[]]
		for beacon_number in range(0, section_split):
			beacon = ctrl.Antecedent(np.arange(0, 7, 0.1), ('distance_beacon%d' % beacon_number))
			
			beacon['close'] = fuzz.trimf(beacon.universe, [0.0, 0.0, 0.7])
			beacon['medium'] = fuzz.trimf(beacon.universe, [0.5, 1.0, 1.5])
			beacon['far'] = fuzz.trapmf(beacon.universe, [1.5, 2.0, 5.5, 5.5])


			rule01_left = ctrl.Rule(beacon['far'], velocityLeft['very fast'])
			rule02_left = ctrl.Rule(beacon['medium'], velocityLeft['fast'])
			rule03_left = ctrl.Rule(beacon['close'], velocityLeft['slow'])
			rulesSectorEast[0] += [rule01_left, rule02_left, rule03_left]

			rule01_right = ctrl.Rule(beacon['far'], velocityRight['slow'])
			rule02_right = ctrl.Rule(beacon['medium'], velocityRight['slow'])
			rule03_right = ctrl.Rule(beacon['close'], velocityRight['very fast'])
			rulesSectorEast[1] += [rule01_right, rule02_right, rule03_right]

		
		# North Sector 
		rulesSectorNorth = [[],[]]
		for beacon_number in range(section_split, 2*section_split):
			beacon = ctrl.Antecedent(np.arange(0, 7, 0.1), ('distance_beacon%d' % beacon_number))
			
			beacon['close'] = fuzz.trimf(beacon.universe, [0.0, 0.0, 0.7])
			beacon['medium'] = fuzz.trimf(beacon.universe, [0.5, 1.0, 1.5])
			beacon['far'] = fuzz.trapmf(beacon.universe, [1.5, 2.0, 5.5, 5.5])


			#rule01_left = ctrl.Rule(beacon['far'], velocityLeft['fast'])
			#rule02_left = ctrl.Rule(beacon['medium'], velocityLeft['slow'])
			rule03_left = ctrl.Rule(beacon['close'], velocityLeft['very slow'])
			#rulesSectorNorth[0] += [rule01_left, rule02_left, rule03_left]
			rulesSectorNorth[0] += [rule03_left]


			#rule01_right = ctrl.Rule(beacon['far'], velocityRight['fast'])
			#rule02_right = ctrl.Rule(beacon['medium'], velocityRight['slow'])
			rule03_right = ctrl.Rule(beacon['close'], velocityRight['very slow'])
			rulesSectorNorth[1] += [rule03_right]

		# West Sector
		rulesSectorWest = [[],[]]
		for beacon_number in range(2*section_split, 3*section_split):
			beacon = ctrl.Antecedent(np.arange(0, 7, 0.1), ('distance_beacon%d' % beacon_number))
			
			beacon['close'] = fuzz.trimf(beacon.universe, [0.0, 0.0, 0.7])
			beacon['medium'] = fuzz.trimf(beacon.universe, [0.5, 1.0, 1.5])
			beacon['far'] = fuzz.trapmf(beacon.universe, [1.5, 2.0, 5.5, 5.5])

			#rule01_left = ctrl.Rule(beacon['far'], velocityLeft['very fast'])
			#rule02_left = ctrl.Rule(beacon['medium'], velocityLeft['fast'])
			rule03_left = ctrl.Rule(beacon['close'], velocityLeft['medium'])
			#rulesSectorWest[0] += [rule01_left, rule02_left, rule03_left]
			rulesSectorWest[0] += [rule03_left]



			#rule01_right = ctrl.Rule(beacon['far'], velocityRight['very fast'])
			#rule02_right = ctrl.Rule(beacon['medium'], velocityRight['slow'])
			rule03_right = ctrl.Rule(beacon['close'], velocityRight['very slow'])
			#rulesSectorWest[1] += [rule01_right, rule02_right, rule03_right]
			rulesSectorWest[1] += [rule03_right]

		

		
		velocityLeftRules = rulesSectorEast[0] + rulesSectorNorth[0] + rulesSectorWest[0]
		velocityRightRules = rulesSectorEast[1] + rulesSectorNorth[1] + rulesSectorWest[1]

		print("before Left Control")
		velLeftCtrl = ctrl.ControlSystem(velocityLeftRules)
		print("before Right Control")
		velRightCtrl = ctrl.ControlSystem(velocityRightRules)

		self.velLeftCtrlSimulation = ctrl.ControlSystemSimulation(velLeftCtrl)
		self.velRightCtrlSimulation = ctrl.ControlSystemSimulation(velRightCtrl)
		

	def getVelocities(self, distances):

		for beacon_number in range(63):
			self.velLeftCtrlSimulation.input[('distance_beacon%d' % beacon_number)] = distances[beacon_number]
			self.velRightCtrlSimulation.input[('distance_beacon%d' % beacon_number)] = distances[beacon_number]
			

		self.velLeftCtrlSimulation.compute()
		self.velRightCtrlSimulation.compute()

		left_velocity = self.velLeftCtrlSimulation.output['velocity left']
		right_velocity = self.velRightCtrlSimulation.output['velocity right']

		return left_velocity, right_velocity


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

	

		

		


		



