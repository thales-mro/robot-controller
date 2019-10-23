import sys
sys.path.insert(0, 'src')
sys.path.insert(0, 'lib')
from robot import Robot
import numpy as np
from numpy.linalg import norm
import matplotlib.pyplot as plt
import time
import vrep
import argparse

parser = argparse.ArgumentParser()
arg = parser.add_argument

arg('--mode', type=int)
arg('--circuit', type=int)
arg('--filtering', type=bool)

args = parser.parse_args()

# Select the Mode, Circuit and Filtering
# Mode can be 1, 2 or 3 (encoders, gyroscope or fusion)
MODE = int(args.mode)

#Circuit: 1,2,3. One of the possible trajectories shown in the report
CIRCUIT = int(args.circuit)

# 0 or 1. A boolean value. It is used to decide if we multiply or not the angle by a factor.
FILTER = bool(args.filtering)


 	  	

def main():

	

	robot = Robot()

	if CIRCUIT == 1:
		Circuit01(robot)

	elif CIRCUIT == 2:
		Circuit02(robot)

	elif CIRCUIT == 3:
		Circuit03(robot)

	else:
		print("Please, choose a valid circuit number: 1, 2 or 3!")
		exit()



def Circuit01(robot):

	all_x = []
	all_y = []
	robot_trajectory = []
	

	x0,y0,z0 = robot.get_current_position()
	
	# Initialization of the Odometry and trajectory values
	orientation_odometry = 0
	odometry_trajectory = [[x0,y0]]
	moviment_state = [x0, y0, orientation_odometry]

	'''
	Following we basically have two functions: forward and rotate
	
	forward performs a straight line given in the second possition. The arguments are:
	robot: the robot instantiation
	dist: distance to be covered by the robot
	all_x: all x values of poinst extracted from the enviroment 
	all_y: all x values of poinst extracted from the enviroment 
	robot_trajectory: the ground truth of the robot trajectory 
	moviment_state: the vector (x,y,theta) of the last iteration of the odometry 
	odometry_trajectory: the predicted trajectory of robot so far given by odometry

	rotate performs the roation of the robot. It is necessary to set the velocities of the wheels
	before calling the function. Almost all of the arguments are equivalent to forward arguments. 
	Only two are diferent: 
	angle: the angle, in radians, to rotate the robot  
	orientation: define for each orientation the robot turns, left (1) or right (0)

	'''

	# Hard-coded trajectory 
	if(robot.get_connection_status() != -1):
		
		

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 1.0, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory) # 3.15
	
		robot.set_left_velocity(2.0)
		robot.set_right_velocity(0.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(45.0), 0, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		
		
		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 2.0, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory) # 4.6

		
		
		robot.set_left_velocity(0.0)
		robot.set_right_velocity(2.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(90.0), 1, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 3.0, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory) # 4.6

		robot.set_left_velocity(0.0)
		robot.set_right_velocity(2.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(120.0), 1, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 2.0, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory) # 4.6

		
		odometry_trajectory = np.array(odometry_trajectory)
		robot_trajectory = np.array(robot_trajectory)
		
		print(np.array(robot_trajectory).shape)
		
		# Once everything has been calculated, it plots and saves them
		plt.plot(-1*odometry_trajectory[1:,0], -1*odometry_trajectory[1:,1], '.')
		plt.plot(-1*robot_trajectory[1:,0], -1*robot_trajectory[1:,1], 'g.')

		plt.plot(-1*odometry_trajectory[:1,0], -1*odometry_trajectory[:1,1], 'r.')
		plt.plot(-1*robot_trajectory[:1,0], -1*robot_trajectory[:1,1], 'r.')

		plt.show()

		np.save("TrajectoryM" + str(MODE) + "C1.npy", robot_trajectory, fix_imports=False)
		np.save("OdometryM" + str(MODE) + "C1.npy", odometry_trajectory, fix_imports=False)




def Circuit02(robot):

	all_x = []
	all_y = []
	robot_trajectory = []
	
	# Initialization of the Odometry and trajectory values
	x0,y0,z0 = robot.get_current_position()
	
	orientation_odometry = 0
	odometry_trajectory = [[x0,y0]]
	moviment_state = [x0, y0, orientation_odometry]

	'''
	Following we basically have two functions: forward and rotate
	
	forward performs a straight line given in the second possition. The arguments are:
	robot: the robot instantiation
	dist: distance to be covered by the robot
	all_x: all x values of poinst extracted from the enviroment 
	all_y: all x values of poinst extracted from the enviroment 
	robot_trajectory: the ground truth of the robot trajectory 
	moviment_state: the vector (x,y,theta) of the last iteration of the odometry 
	odometry_trajectory: the predicted trajectory of robot so far given by odometry

	rotate performs the roation of the robot. It is necessary to set the velocities of the wheels
	before calling the function. Almost all of the arguments are equivalent to forward arguments. 
	Only two are diferent: 
	angle: the angle, in radians, to rotate the robot  
	orientation: define for each orientation the robot turns, left (1) or right (0)

	'''

	
	# Hard-coded trajectory 
	if(robot.get_connection_status() != -1):
		
		

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 1.0, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory) # 3.15
	
		robot.set_left_velocity(0.0)
		robot.set_right_velocity(2.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(85.0), 1, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		
		
		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 1.0, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory) # 4.6

		
		
		robot.set_left_velocity(0.0)
		robot.set_right_velocity(2.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(85.0), 1, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 1.0, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory) # 4.6

		robot.set_left_velocity(0.0)
		robot.set_right_velocity(2.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(85.0), 1, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 1.0, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory) # 4.6

		
		odometry_trajectory = np.array(odometry_trajectory)
		robot_trajectory = np.array(robot_trajectory)
		
		print(np.array(robot_trajectory).shape)
		
		# once the code has all the information, they are plotted and saved
		plt.plot(-1*odometry_trajectory[1:,0], -1*odometry_trajectory[1:,1], '.')
		plt.plot(-1*robot_trajectory[1:,0], -1*robot_trajectory[1:,1], 'g.')

		plt.plot(-1*odometry_trajectory[:1,0], -1*odometry_trajectory[:1,1], 'r.')
		plt.plot(-1*robot_trajectory[:1,0], -1*robot_trajectory[:1,1], 'r.')

		plt.show()

		np.save("TrajectoryM" + str(MODE) + "C2.npy", robot_trajectory, fix_imports=False)
		np.save("OdometryM" + str(MODE) + "C2.npy", odometry_trajectory, fix_imports=False)
	


def Circuit03(robot):

	all_x = []
	all_y = []
	robot_trajectory = []
	
	# Initialization of the Odometry and trajectory values
	x0,y0,z0 = robot.get_current_position()
	
	orientation_odometry = 0
	odometry_trajectory = [[x0,y0]]
	moviment_state = [x0, y0, orientation_odometry]

	'''
	Following we basically have two functions: forward and rotate
	
	forward performs a straight line given in the second possition. The arguments are:
	robot: the robot instantiation
	dist: distance to be covered by the robot
	all_x: all x values of poinst extracted from the enviroment 
	all_y: all x values of poinst extracted from the enviroment 
	robot_trajectory: the ground truth of the robot trajectory 
	moviment_state: the vector (x,y,theta) of the last iteration of the odometry 
	odometry_trajectory: the predicted trajectory of robot so far given by odometry

	rotate performs the roation of the robot. It is necessary to set the velocities of the wheels
	before calling the function. Almost all of the arguments are equivalent to forward arguments. 
	Only two are diferent: 
	angle: the angle, in radians, to rotate the robot  
	orientation: define for each orientation the robot turns, left (1) or right (0)

	'''
	
	# Hard-coded trajectory 
	if(robot.get_connection_status() != -1):
		
		
		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 3.15, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory) # 3.15

		
		robot.set_left_velocity(0.0)
		robot.set_right_velocity(2.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(82.0), 1, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		
		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 4.6, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory) # 4.6

		
		robot.set_left_velocity(2.0)
		robot.set_right_velocity(0.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(90.0), 0, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 1.5, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory) # 4.6

		robot.set_left_velocity(2.0)
		robot.set_right_velocity(0.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(180.0), 1, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 1.0, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory)
		
		
		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 0.3, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory)
		

		robot.set_left_velocity(0.0)
		robot.set_right_velocity(2.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(95.0), 1, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82
		

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 4.0, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory)
		
		robot.set_left_velocity(2.0)
		robot.set_right_velocity(0.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(90.0), 0, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 7.0, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory)
		
		

		robot.set_left_velocity(2.0)
		robot.set_right_velocity(0.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(75.0), 0, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 3.5, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory)
		

		
		robot.set_left_velocity(0.0)
		robot.set_right_velocity(2.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(180.0), 1, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		
		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 4.0, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory)
		

		robot.set_left_velocity(0.0)
		robot.set_right_velocity(2.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(70.0), 1, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 4.0, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory)
		

		robot.set_left_velocity(2.0)
		robot.set_right_velocity(0.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(80.0), 0, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 1.9, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory)
		
		robot.set_left_velocity(2.0)
		robot.set_right_velocity(0.0)
		all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory = rotate(robot, degreesToRadians(80.0), 0, all_x, all_y, robot_trajectory,moviment_state, odometry_trajectory) # 82

		robot.set_left_velocity(3.0)
		robot.set_right_velocity(3.0)
		all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory = forward(robot, 1.2, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory)
		


	delta_space = len(robot_trajectory)//1000
	odometry_trajectory = np.array(odometry_trajectory)[::delta_space]
	robot_trajectory = np.array(robot_trajectory)[::delta_space]
	pointsToSave = np.array([all_x, all_y])

	print(np.array(robot_trajectory).shape)
	
	# Once the informations have been gotten, the process plot and save them
	plt.plot(-1*np.array(all_x), -1*np.array(all_y), 'o')
	plt.show()

	plt.plot(-1*odometry_trajectory[1:,0], -1*odometry_trajectory[1:,1], '.')
	plt.plot(-1*robot_trajectory[1:,0], -1*robot_trajectory[1:,1], 'g.')

	plt.plot(-1*odometry_trajectory[:1,0], -1*odometry_trajectory[:1,1], 'r.')
	plt.plot(-1*robot_trajectory[:1,0], -1*robot_trajectory[:1,1], 'r.')

	plt.show()

	np.save("ExtractedPoints.npy", pointsToSave, fix_imports=False)
	np.save("TrajectoryM" + str(MODE) + "C3.npy", robot_trajectory, fix_imports=False)
	np.save("OdometryM" + str(MODE) + "C3.npy", odometry_trajectory, fix_imports=False)


def degreesToRadians(degree_angle):
	radian_angle = degree_angle*np.pi/180
	return radian_angle


# Forward function performs a straight line given the distance. While the robot is moving
# all the informations from the lasers are got and transformed to the global frame or
# stored in the cumulators of the ground truth, predicted trajectory or points extracted
def forward(robot, dist, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory):

	# Before starting, we get the current possition
	stop = False
	ref0_pos = np.array(robot.get_current_position())


	while(robot.get_connection_status() != -1 and not stop):
		ir_distances = robot.read_laser()
		ir_distances = np.array(ir_distances).reshape(len(ir_distances)//3,3)[::150]

		robot_angles = robot.get_current_orientation()
		theta = robot_angles[2]

		plot_rot_x, plot_rot_y = rotation(theta, ir_distances[:,0], ir_distances[:,1])
		robot_pos = robot.get_current_position()
		plot_x, plot_y = translation(robot_pos[0], robot_pos[1], plot_rot_x, plot_rot_y)

		robot_trajectory.append([robot_pos[0], robot_pos[1]])
		## ---- Odometry ---- ##
		x, y, orientation_odometry = odometry(robot, moviment_state[0], moviment_state[1], moviment_state[2])
		odometry_trajectory.append([x,y])

		moviment_state[0] = x
		moviment_state[1] = y
		moviment_state[2] = orientation_odometry

		## ------------------- ##

		all_x.extend(plot_x)
		all_y.extend(plot_y)	

		ref1_pos = np.array(robot.get_current_position())

		# The final position is always checked compared to the initial one
		if norm(ref1_pos - ref0_pos) >= dist:
			robot.set_left_velocity(0.0)
			robot.set_right_velocity(0.0)
			stop = True


	return all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory



		

# Rotate function performs a rotation of the robot given the angle and orientation of the rotation. 
# While the robot is moving all the informations from the lasers are got and transformed to the global frame or
# stored in the cumulators of the ground truth, predicted trajectory or Extracted Points.
def rotate(robot, angle, orientation, all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory):

	stop = False
	robot_angles = robot.get_current_orientation()
	ref0_theta = robot_angles[2]

	while(robot.get_connection_status() != -1 and not stop):
		ir_distances = robot.read_laser()
		ir_distances = np.array(ir_distances).reshape(len(ir_distances)//3,3)[::150]

		robot_angles = robot.get_current_orientation()
		theta = robot_angles[2]

		plot_rot_x, plot_rot_y = rotation(theta, ir_distances[:,0], ir_distances[:,1])
		robot_pos = robot.get_current_position()
		plot_x, plot_y = translation(robot_pos[0], robot_pos[1], plot_rot_x, plot_rot_y)

		robot_trajectory.append([robot_pos[0], robot_pos[1]])

		## ---- Odometry ---- ##
		x, y, orientation_odometry = odometry(robot, moviment_state[0], moviment_state[1], moviment_state[2])
		#print(x,y, 'r')
		odometry_trajectory.append([x,y])

		moviment_state[0] = x
		moviment_state[1] = y
		moviment_state[2] = orientation_odometry

		## ------------------- ##

		all_x.extend(plot_x)
		all_y.extend(plot_y)	

		robot_angles = robot.get_current_orientation()
		ref1_theta = robot_angles[2]

		# Robot rotates to right
		if orientation == 0:

			if ref0_theta <= 0.0 and ref1_theta >= 0.0:

				if (ref0_theta + np.pi) + (np.pi - ref1_theta) >= angle:
					robot.set_left_velocity(0.0)
					robot.set_right_velocity(0.0)
					stop = True

			elif abs(ref1_theta - ref0_theta) >= angle:
				robot.set_left_velocity(0.0)
				robot.set_right_velocity(0.0)
				stop = True
			
		#Robot rotates to left
		elif orientation == 1:

			if ref0_theta >= 0.0 and ref1_theta <= 0.0:
				#print(np.pi - ref0_theta, ref1_theta + np.pi, angle)
				if (ref1_theta + np.pi) + (np.pi - ref0_theta) >= angle:
					robot.set_left_velocity(0.0)
					robot.set_right_velocity(0.0)
					stop = True


			elif abs(ref1_theta - ref0_theta) >= angle:
				robot.set_left_velocity(0.0)
				robot.set_right_velocity(0.0)
				stop = True


	return all_x, all_y, robot_trajectory, moviment_state, odometry_trajectory


# Funtion that calculate the variations DeltaX, DeltaY and DeltaTheta
def odometry(robot, x, y, orientation):
	# read initial angles from each wheel
	angle0_left = get_left_encoder(robot)
	angle0_right = get_right_encoder(robot)

	# read theta variation from gyroscope
	res, gyroZ = vrep.simxGetFloatSignal(robot.clientID, "gyroZ", vrep.simx_opmode_streaming)
	
	# wait for a time to wheel turns a little bit
	time.sleep(0.1)

	# read final angles from each wheel
	angle1_left = get_left_encoder(robot)
	angle1_right = get_right_encoder(robot)


	# Deal with the discontinuity of the angles
	if angle0_left > 0.0 and angle1_left < 0.0:
		dtheta_left = (angle1_left + np.pi) + (np.pi - angle0_left)
	else:
		dtheta_left = abs(angle1_left - angle0_left)

	if angle0_right > 0.0 and angle1_right < 0.0:
		dtheta_right = (angle1_right + np.pi) + (np.pi - angle0_right)
	else:
		dtheta_right = abs(angle1_right - angle0_right) 


	# Performs the comparison statment to decide which mode and Filter state the 
	# robot needs to follow to get Delta Theta
	if MODE == 1:
		dangle_encoder = ((robot.WHEEL_RADIUS*(dtheta_right - dtheta_left))/robot.ROBOT_WIDTH)

		if FILTER:
			dangle = dangle_encoder*1.05
		else:
			dangle = dangle_encoder

	elif MODE == 2:

		if FILTER:
			dangle = gyroZ*0.85
		else:
			dangle = gyroZ
		

	elif MODE == 3:
		dangle_encoder = (robot.WHEEL_RADIUS*(dtheta_right - dtheta_left))/robot.ROBOT_WIDTH #-- encoder odometry

		if FILTER:
			dangle = (0.9*gyroZ + 1.05*dangle_encoder)/2
		else: 
			dangle = (gyroZ + dangle_encoder)/2 


	# Set a threshold to avoid accumulate erros while robot is getting data 
	# for the odometry
	if abs(dangle) < 0.01:
		dangle = 0.0

	if dtheta_right < 0.01:
		dtheta_right = 0.0

	if dtheta_left < 0.01:
		dtheta_left = 0.0

	# Get the x and y variations
	ds = (robot.WHEEL_RADIUS*(dtheta_right + dtheta_left))/2

	dx = ds*np.cos(orientation + dangle/2)
	dy = ds*np.sin(orientation + dangle/2)

	return x+dx, y+dy, orientation+dangle


# The two folloing fucntions are used to get the reading from the angle detetcion of the wheels. We treat them
# as encoders since they behave very similar to one enconder.
def get_left_encoder(robot):
	values = vrep.simxGetJointPosition(robot.clientID, robot.motors_handle["left"], vrep.simx_opmode_streaming)
	return values[1]
	
def get_right_encoder(robot):
	values = vrep.simxGetJointPosition(robot.clientID, robot.motors_handle["right"], vrep.simx_opmode_streaming)
	return values[1]

# function to adjust the angles
def getAngle(angle):

	if angle < 0:
		angle = angle + 2*np.pi

	return angle

# Get the point (x,y) in a rotated frame
def rotation(theta, x, y):
	rotated_x = np.cos(theta)*x - np.sin(theta)*y
	rotated_y = np.sin(theta)*x + np.cos(theta)*y
	return rotated_x, rotated_y

# Get the point (x,y) in a translated frame
def translation(robot_x, robot_y, x, y):
	translated_x = robot_x + x
	translated_y = robot_y + y
	return translated_x, translated_y 


if __name__ == '__main__':
	main()