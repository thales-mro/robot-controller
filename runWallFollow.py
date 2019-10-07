from wallFollow import WallFollowController
import sys
sys.path.insert(0, 'src')
sys.path.insert(0, 'lib')
from robot import Robot
import time

def main():
    """
    Entrypoint for wallFollow controller
    """
    robot = Robot()
    controller = WallFollowController()

    while robot.get_connection_status() != -1:
        dist = robot.read_ultrassonic_sensors()
        velocities = controller.control(dist[8])
        robot.set_left_velocity(velocities[0])
        robot.set_right_velocity(velocities[1])
        if dist[8] != 5.0:
            print(dist[8])
        time.sleep(0.1)

if __name__ == '__main__':
    main()
