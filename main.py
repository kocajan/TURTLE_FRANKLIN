import yaml

from src import Robot, get_to_gate, park
import time as time


def main():
    # Load configuration files
    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))
    move_cfg = yaml.safe_load(open('conf/move.yaml', 'r'))

    # Create robot object
    robot_cfg = objects_cfg['robot']
    rob = Robot(robot_cfg['radius'], robot_cfg['height'], robot_cfg['color'])
    rob.set_world_coordinates(robot_cfg['world_coordinates'])

    # Wait for button to start
    while not rob.get_start():
        time.sleep(1)

    # Get to the gate
    get_to_gate(rob, detection_cfg, objects_cfg, move_cfg)

    # Park the robot
    park(rob, detection_cfg, objects_cfg, move_cfg)

    # Play sound when parked
    rob.end_music()

if __name__ == '__main__':
    main()
