import numpy as np
import time

from numpy.linalg import norm

class WallFollowController():
    """
    It implements methods for wall-follow-type controller using PID logic.
    It assumes the wall to be followed is at its right or left.
    """

    def __init__(self, robot):
        self.left_wheel_speed = 1.5 # medium base speed for wheels
        self.right_wheel_speed = 1.5

        self.k_p = 2.0 # empirical value
        self.k_i = 0.0001 # empirical value
        self.k_d = 20 #empirical value
        self.reference = 0.30 # in centimeters
        self.integral_error = 0.0
        self.last_step_error = 0.0


    def reset_integral_error(self):
        """
        It zeroes integral error (it can be used for control re-execution)
        """
        self.integral_error = 0.0

    def p_control(self, sensor_distance, direction='right'):
        """
        It performs proportional control for wall following behavior

        Keyword arguments:
        sensor_distance -- the measured distance from sensor of interest
        direction -- direction related to which US sensor is analyzed
            right (default): US sensor positioned at robot's right side
            left: US sensor positioned at robot's left side
        """
        if sensor_distance >= 5.0:
            return (0, 0)

        error = self.reference - sensor_distance
        if direction == 'right':
            right_vel = (error * self.k_p) + self.right_wheel_speed
            left_vel = self.left_wheel_speed
            print("Speed", right_vel, direction)
        else:
            left_vel = (error * self.k_p) + self.left_wheel_speed
            right_vel = self.right_wheel_speed
            print("Speed", left_vel, direction)


        return (left_vel, right_vel)


    def pi_control(self, sensor_distance, direction='right'):
        """
        It performs proportional-integrative control for wall following behavior

        Keyword arguments:
        sensor_distance -- the measured distance from sensor of interest
        direction -- direction related to which US sensor is analyzed
            right (default): US sensor positioned at robot's right side
            left: US sensor positioned at robot's left side
        """
        if sensor_distance >= 5.0:
            return (0, 0)

        error = self.reference - sensor_distance
        self.integral_error += error

        if direction == 'right':
            right_vel = (error * self.k_p) + (
                self.integral_error * self.k_i) + self.right_wheel_speed
            left_vel = self.left_wheel_speed
            print("Speed", right_vel)
        else:
            left_vel = (error * self.k_p) + (self.integral_error * self.k_i) + self.left_wheel_speed
            right_vel = self.right_wheel_speed
            print("Speed", left_vel)

        return (left_vel, right_vel)

    def pd_control(self, sensor_distance, direction='right'):
        """
        It performs proportional-derivative control for wall following behavior

        Keyword arguments:
        sensor_distance -- the measured distance from sensor of interest
        direction -- direction related to which US sensor is analyzed
            right (default): US sensor positioned at robot's right side
            left: US sensor positioned at robot's left side
        """
        if sensor_distance >= 5.0:
            return (0, 0)

        error = self.reference - sensor_distance
        diff_error = error - self.last_step_error
        self.last_step_error = error

        if direction == 'right':
            right_vel = (error * self.k_p) + (diff_error * self.k_d) + self.right_wheel_speed
            left_vel = self.left_wheel_speed
            print("Speed", right_vel)
        else:
            left_vel = (error * self.k_p) + (diff_error * self.k_d) + self.left_wheel_speed
            right_vel = self.right_wheel_speed
            print("Speed", left_vel)

        return (left_vel, right_vel)

    def pid_control(self, sensor_distance, direction='right'):
        """
        It performs proportional-derivative control for wall following behavior

        Keyword arguments:
        sensor_distance -- the measured distance from sensor of interest
        direction -- direction related to which US sensor is analyzed
            right (default): US sensor positioned at robot's right side
            left: US sensor positioned at robot's left side
        """
        if sensor_distance >= 5.0:
            return (0, 0)

        error = self.reference - sensor_distance
        diff_error = error - self.last_step_error
        self.last_step_error = error
        self.integral_error += error

        if direction == 'right':
            right_vel = (error * self.k_p) + (
                diff_error * self.k_d) + (self.integral_error * self.k_i) + self.right_wheel_speed
            left_vel = self.left_wheel_speed
            print("Speed", right_vel)
        else:
            left_vel = (error * self.k_p) + (
                diff_error * self.k_d) + (self.integral_error * self.k_i) + self.left_wheel_speed
            right_vel = self.right_wheel_speed
            print("Speed", left_vel)

        return (left_vel, right_vel)

      

    ## Function made by Gabriel only in the feature/State_machine branch 
    def getStoredAngles(self):
        return np.load("stored_angles.npy")

    ## Function made by Gabriel only in the feature/State_machine branch 
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

    ## Function made by Gabriel only in the feature/State_machine branch 
    def run(self, sensor_number, sensor_side):

        half = 684//2
        while self.robot.get_connection_status() != -1:


            ir_distances = self.robot.read_laser()
            ir_distances = np.array(ir_distances).reshape(len(ir_distances)//3,3)[:684,:2]

            r, theta = self.getInputValues(ir_distances)

            if sensor_side == "right":
                detection_range = r[half:half+10]
            elif sensor_side == "left":
                detection_range = r[half-10:half]

            min_dist = np.min(detection_range)

            if min_dist >= 0.75:
                dist = self.robot.read_ultrassonic_sensors()
                velocities = self.control(dist[sensor_number], sensor_side)
                self.robot.set_left_velocity(velocities[0])
                self.robot.set_right_velocity(velocities[1])
                if dist[sensor_number] >= 5.0:
                    print(dist[sensor_number])
                    return 1
                time.sleep(0.1)

            else:
                return 2

