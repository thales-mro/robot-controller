import numpy as np
from numpy.linalg import norm
import skfuzzy as fuzz
from skfuzzy import control as ctrl
import matplotlib.pyplot as plt
import scipy.stats
import time
import vrep



class avoidObstacleFuzzyController():


	def __init__(self, robot, odometry):
		self.robot = robot
		self.odometry = odometry
		self.defined_finish = None
		self.velCtrlLeftWheel = []
		self.velCtrlRightWheel = []
		self.sensor_side = None
		self.sensor_number = None
		#self.velocity_weights = np.exp(-1*np.arange(0.01, 3.43, 0.01)[:-1][::-1])
		self.velocity_weights = scipy.stats.norm(242.5, 90.0).pdf(range(342))
		self.sum_of_weights = np.sum(self.velocity_weights)
		print(self.velocity_weights)


		# It defines the membership functions to each of the 5 levels of velocity.
		# Membership functions are set to both wheels.
		velocityRight = ctrl.Consequent(np.arange(0, 5.5, 0.1), 'velocity right')
		velocityLeft = ctrl.Consequent(np.arange(0, 5.5, 0.1), 'velocity left')

		velocityRight['very slow'] = fuzz.trimf(velocityRight.universe, [0.0, 0.0, 1.0])
		velocityRight['slow'] = fuzz.trimf(velocityRight.universe, [0.8, 1.5, 2.2])
		velocityRight['medium'] = fuzz.trimf(velocityRight.universe, [2.0, 2.7, 3.4])
		velocityRight['fast'] = fuzz.trimf(velocityRight.universe, [3.2, 3.9, 4.6])
		velocityRight['very fast'] = fuzz.trimf(velocityRight.universe, [4.4, 5.1, 5.1])

		

		velocityLeft['very slow'] = fuzz.trimf(velocityLeft.universe, [0.0, 0.0, 1.0])
		velocityLeft['slow'] = fuzz.trimf(velocityLeft.universe, [0.8, 1.5, 2.2])
		velocityLeft['medium'] = fuzz.trimf(velocityLeft.universe, [2.0, 2.7, 3.4])
		velocityLeft['fast'] = fuzz.trimf(velocityLeft.universe, [3.2, 3.9, 4.6])
		velocityLeft['very fast'] = fuzz.trimf(velocityLeft.universe, [4.4, 5.1, 5.1])

		# East Sector
		#section_split = 63//3
		#self.section_split = 21
		self.section_split = 684//2

		'''
		For each beacon, it is defined a membership function for three levels of distance:
		'close', 'medium', 'far'. The implication is stablished between this levels and the
		velocities acording to the wheel side. For each beacon we have a fuzzy logic associated. 
		We use half of the laser beacon (all of the right side) in order to decide to which side
		to go in case of avoiding obstacle. So, for each beacon, we have a fuzzy system to each wheel.  
		'''
		for beacon_number in range(0, self.section_split):
			beacon = ctrl.Antecedent(np.arange(0, 7, 0.1), ('distance_beacon%d' % beacon_number))
			
			beacon['close'] = fuzz.trimf(beacon.universe, [0.0, 0.0, 1.0])
			beacon['medium'] = fuzz.trimf(beacon.universe, [0.8, 1.3, 1.8])
			beacon['far'] = fuzz.trapmf(beacon.universe, [1.6, 2.1, 5.5, 5.5])


			rule01_left = ctrl.Rule(beacon['far'], velocityLeft['very fast'])
			rule02_left = ctrl.Rule(beacon['medium'], velocityLeft['fast'])
			rule03_left = ctrl.Rule(beacon['close'], velocityLeft['very slow'])

			rule01_right = ctrl.Rule(beacon['far'], velocityRight['very slow'])
			rule02_right = ctrl.Rule(beacon['medium'], velocityRight['slow'])
			rule03_right = ctrl.Rule(beacon['close'], velocityRight['very fast'])

			velLeftCtrl = ctrl.ControlSystem([rule01_left, rule02_left, rule03_left])
			velRightCtrl = ctrl.ControlSystem([rule01_right, rule02_right, rule03_right])

			self.velCtrlLeftWheel.append(ctrl.ControlSystemSimulation(velLeftCtrl))
			self.velCtrlRightWheel.append(ctrl.ControlSystemSimulation(velRightCtrl))

		

		
		

		

	def getVelocities(self, distances):

		left_velocity = 0.0
		right_velocity = 0.0

		# Each beacon gives a distance which is used to calculate the velocity.
		# Then, each beacon will give a velocity. All of the are weighted averaged.
		# A gaussian distribution is associated to the velocities. 
		for beacon_number in range(self.section_split):
			
			self.velCtrlLeftWheel[beacon_number].input[('distance_beacon%d' % beacon_number)] = distances[beacon_number]
			self.velCtrlRightWheel[beacon_number].input[('distance_beacon%d' % beacon_number)] = distances[beacon_number]
			
			self.velCtrlLeftWheel[beacon_number].compute()
			self.velCtrlRightWheel[beacon_number].compute()

			vl = self.velCtrlLeftWheel[beacon_number].output['velocity left']
			vr = self.velCtrlRightWheel[beacon_number].output['velocity right']

			# Weighted averaging
			left_velocity += vl*self.velocity_weights[beacon_number]
			right_velocity += vr*self.velocity_weights[beacon_number]
			
			if beacon_number % 97 == 0:
				self.odometry.calculate_odometry()
		
		mean_letf_velocity = left_velocity/self.sum_of_weights
		mean_right_velocity = right_velocity/self.sum_of_weights
		
		return mean_letf_velocity, mean_right_velocity


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

	def run(self, trajectory):

		half = 684//2
		half_section = 21//2
		threshold = np.pi/3 
		
		if(self.robot.get_connection_status() != -1):
			angle_ref = self.odometry.get_pose()[-1]

		while(self.robot.get_connection_status() != -1):
			#self.odometry.calculate_odometry()
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

			if min_dist <= 0.80: # Verify if the robot is free to go forward
				r1 = r[:half]	
				r_downsampled = r1

				velLeft, velRight = self.getVelocities(r_downsampled)

				#print(velLeft, velRight)
				self.robot.set_left_velocity(velLeft)
				self.robot.set_right_velocity(velRight)
				#time.sleep(0.1)

			else: # After change the orientation, a new provisory goal is set.
				angle_final = self.odometry.get_pose()[-1]
				actual_pos = self.odometry.get_pose()[:2]

				r = 1.5

				# Since we do not previous know if the robot is avoiding a obstacle or a wall
				# We set the sensor side and the sensor number if the former option happens.
				if angle_ref <= -np.pi/2 and angle_final >= np.pi/2: 
					dtheta = angle_ref - angle_final + 2*np.pi
					self.sensor_side = "left"
					self.sensor_number = 15

				elif angle_final <= -np.pi/2 and angle_ref >= np.pi/2: 
					dtheta = angle_final - angle_ref + 2*np.pi
					self.sensor_side = "right"
					self.sensor_number = 8

				else:
					dtheta = abs(angle_final - angle_ref)

					if angle_final > angle_ref:
						self.sensor_side = "right"
						self.sensor_number = 7
					else:
						self.sensor_side = "left"
						self.sensor_number = 0

				# Setting new provisory goal
				self.defined_finish = np.array([[r*np.cos(angle_final) + actual_pos[0], r*np.sin(angle_final) + actual_pos[1]]])
				
				if dtheta < threshold:
					return 1, trajectory
				else:
					return 3, trajectory


	def getDefinedFinish(self):
		return self.defined_finish


				

	

		

		


		


