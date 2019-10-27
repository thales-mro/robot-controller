from wallFollow import WallFollowController
import sys
import time
import numpy as np
import matplotlib.pyplot as plt
sys.path.insert(0, 'src')
sys.path.insert(0, 'lib')
from robot import Robot
from odometry import Odometry

def main():
    """
    Entrypoint for wallFollow controller.
    It should be used to test the isolated behavior.
    """
    robot = Robot()
    x, y = robot.get_current_position()[0:2]
    orientation = robot.get_current_orientation()[2]
    odom = Odometry(robot, x, y, orientation, mode=0)
    controller = WallFollowController(robot, odom)

    ts = 0.0
    start = time.time()
    ts_array = []
    sensor_distances = []

    ts_now = time.time()
    while robot.get_connection_status() != -1:
        dist = robot.read_ultrassonic_sensors()

        # Logic for wall follow test 01 and 02
        '''if dist[7] != 5.0:
            ts_array.append(ts)
            sensor_distances.append(dist[7])'''
        # Logic for wall follow test 03
        if dist[0] != 5.0:
            ts_array.append(ts)
            sensor_distances.append(dist[1])

        # Control logic for wall follow test 01 and 02
        # velocities = controller.pid_control(dist[7], 'right')
        # Control logic for wall follow test 03
        velocities = controller.pid_control(dist[1], 'left')
        robot.set_left_velocity(velocities[0])
        robot.set_right_velocity(velocities[1])
        # Stop logic for wall follow test 01 and 02
        '''if dist[7] <= 5.0:
            print(dist[7])
        if dist[3] <= 0.3 or dist[7] == 5.0:
            break'''
        # Stop logic for wall follow test 03
        if dist[1] <= 5.0:
            print(dist[1])
        if dist[4] <= 0.3:
            break
        time.sleep(0.1)

        stop = time.time()
        ts += stop - start
        start = stop

    ref_arr = np.full((len(ts_array)), controller.get_reference())
    print(len(sensor_distances), len(ref_arr))
    plt.plot(ts_array, sensor_distances, 'r', "reference signal", ts_array, ref_arr, 'b--')
    plt.show()
    print("Done!")
    robot.stop()

if __name__ == '__main__':
    main()
