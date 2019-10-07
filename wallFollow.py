import numpy as np

class WallFollowController():
    """
    It implements methods for wall-follow-type controller using PID logic.
    It assumes the wall to be followed is always at its right.
    """

    def __init__(self):
        self.left_wheel_speed = 1.5 # medium base speed for wheels
        self.right_wheel_speed = 1.5
        self.k_p = 2 # empirical value
        self.reference = 0.25 # in centimeters

    def control(self, sensor_distance):
        """
        It performs proportional wall following control

        Keyword arguments:
        sensor_distance -- the measured distance from sensor of interest
        """
        if sensor_distance >= 5.0:
            return (0, 0)

        error = self.reference - sensor_distance
        right_vel = (error * self.k_p) + self.right_wheel_speed
        print("Speed", right_vel)

        return (self.left_wheel_speed, right_vel)

        