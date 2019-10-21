class WallFollowController():
    """
    It implements methods for wall-follow-type controller using PID logic.
    It assumes the wall to be followed is at its right or left.
    """

    def __init__(self):
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
            print("Speed", right_vel)
        else:
            left_vel = (error * self.k_p) + self.left_wheel_speed
            right_vel = self.right_wheel_speed
            print("Speed", left_vel)


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
