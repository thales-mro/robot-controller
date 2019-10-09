import numpy as np

class WallFollowController():
    """
    It implements methods for wall-follow-type controller using PID logic.
    It assumes the wall to be followed is at its right or left.
    """

    def __init__(self):
        self.left_wheel_speed = 1.5 # medium base speed for wheels
        self.right_wheel_speed = 1.5
        self.k_p = 2 # empirical value
        self.reference = 0.25 # in centimeters

    def control(self, sensor_distance, direction='right'):
        """
        It performs proportional wall following control

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
            print("Speed", right_vel)
        else:
            left_vel = (error * self.k_p) + self.left_wheel_speed
            right_vel = self.right_wheel_speed
            print("Speed", left_vel)


        return (left_vel, right_vel)
        