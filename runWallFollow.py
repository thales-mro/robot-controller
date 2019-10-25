from wallFollow import WallFollowController
import sys
sys.path.insert(0, 'src')
sys.path.insert(0, 'lib')
from robot import Robot
from odometry import Odometry
import time

def main():
    """
    Entrypoint for wallFollow controller
    """
    robot = Robot()
    x, y = robot.get_current_position()[0:2]
    orientation = robot.get_current_orientation()[2]
    odom = Odometry(robot, x, y, orientation, mode=0)
    controller = WallFollowController(robot, odom)

    while robot.get_connection_status() != -1:
        dist = robot.read_ultrassonic_sensors()
        # Control logic for wall follow test 02 and 03
        velocities = controller.pid_control(dist[7], 'right')
        # Control logic for wall follow test 04
        # velocities = controller.pid_control(dist[1], 'left')
        robot.set_left_velocity(velocities[0])
        robot.set_right_velocity(velocities[1])
        # Stop logic for wall follow test 02 and 03
        if dist[7] <= 5.0:
            print(dist[7])
        if dist[3] <= 0.3 or dist[7] == 5.0:
            break
        # Stop logic for wall follow test 04
        '''if dist[1] <= 5.0:
            print(dist[1])
        if dist[4] <= 0.3:
            break'''
        time.sleep(0.1)
    print("Done!")
    robot.stop()

if __name__ == '__main__':
    main()
