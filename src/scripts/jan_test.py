import numpy as np
import cv2 as cv
import yaml
import time

import move

from objects import Obstacle, Gate, Garage
from robot import Robot
from detector import Detector
from map import Map
from visualizer import Visualizer


def world_analysis(rob, detection_cfg, objects_cfg, visualize=False, fill_map=True):
    """
    Function that takes image and point cloud from the robot and extracts information about the surrounding world.
    :param rob: Robot object
    :param detection_cfg: Configuration file for detection
    :param objects_cfg: Configuration file for objects
    :param visualize: Boolean value that determines if the process should be visualized
    :param fill_map: Boolean value that determines if the map should be filled with information
    :return: The map, number of pillars of the gate and goal object
    """
    # Load map parameters
    map_dimensions = detection_cfg['map']['dimensions']
    map_resolution = detection_cfg['map']['resolution']

    # Take image and point cloud
    img = rob.take_rgb_img()
    pc = rob.take_point_cloud()

    # Create map object
    map = Map(map_dimensions, map_resolution, detection_cfg)
    map.set_robot(rob)

    # Create detector object
    det = Detector(map, img, pc, detection_cfg, objects_cfg)

    # Process image and point cloud
    det.process_rgb()
    det.process_point_cloud()

    # Extract information from the map
    if fill_map:
        map.fill_world_map()

    # Get information to return
    gate = map.get_gate()
    if gate is not None:
        number_gate_pillars = gate.get_num_pillars()
    else:
        number_gate_pillars = 0

    goal = map.get_goal()
    path = None

    if goal is not None:
        # Select search algorithm
        search_algorithm = detection_cfg['map']['search_algorithm']
        start_point = detection_cfg['map']['start_point']

        # Calculate path from start to goal
        path = map.find_way(start_point, tuple(goal), search_algorithm)

    if visualize:
        # Initialize visualizer object
        vis = Visualizer(img, pc, map, det.get_processed_rgb(), det.get_processed_point_cloud(), detection_cfg)

        vis.visualize_rgb()
        # vis.visualize_point_cloud()
        if goal is not None:
            vis.visualize_map(path=path)
        else:
            vis.visualize_map()

    return map, number_gate_pillars, goal, path


def automate_test() -> None:
    # Load configuration files
    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))

    # Create robot object (it won't be changed during the whole process)
    robot_cfg = objects_cfg['robot']
    rob = Robot(robot_cfg['radius'], robot_cfg['height'], robot_cfg['color'])
    rob.set_world_coordinates(robot_cfg['world_coordinates'])

    # Create small rotation move object (used for small rotations during the searching process)
    small_rot_move = move.Move(rob, None, None)

    # STATE AUTOMAT
    while True:
        # Extract information from the surrounding world
        map, number_gate_pillars, goal, _ = world_analysis(rob, detection_cfg, objects_cfg)

        # If no goal is set (robot does not see the garage [yellow] nor the gate [magenta]) then rotate and search
        if goal is None:
            # Rotate the robot
            small_rot_move.execute_small_rot_positive(20, 0.9)

            # Continue to search for the gate or the garage
            continue
        # We have found the garage/gate, try to improve the robot's position to make the goal easier to set
        # If the goal is set then there are two options:
        else:
            # The robot sees one pillar of the gate
            if number_gate_pillars == 1:
                # Try to find the second pillar
                find_more_pillars(rob, small_rot_move, map, number_gate_pillars, detection_cfg, objects_cfg)
            # The robot sees the garage but not the gate
            elif number_gate_pillars == 0:
                # Try to find the best position to see the gate
                find_best_position_to_see_garage(rob, small_rot_move, map, number_gate_pillars, detection_cfg,
                                               objects_cfg)
        # END OF THE STATE AUTOMAT
        # -> find the best path to the goal and follow it

        # Analyze the world and find the best path and wait for the robot to stop
        time.sleep(0.5)
        map, number_gate_pillars, goal, path = world_analysis(rob, detection_cfg, objects_cfg, visualize=False)

        # Follow the path
        tmp = move.Move(rob, path, detection_cfg)
        tmp.execute_move()

        if not rob.get_stop():
            if map.get_goal_type() == detection_cfg['map']['goal_type']['two_pillars']:
                # All conditions are met, we can start the parking sequence
                break
        else:
            # Robot has stopped, we need to find the path again (and reset stop flag)
            rob.set_stop(False)
    
    # parking sequence


def find_more_pillars(rob, small_rot_move, map, number_gate_pillars, detection_cfg, objects_cfg):
    """
    When the robot sees one gate pillar, there is a possibility that it will see the second one if it rotates.
    This function will rotate the robot to find the second gate pillar.
    :param rob: Robot object
    :param small_rot_move: move object
    :param map: Map object
    :param number_gate_pillars: Number of the gate pillars
    :param detection_cfg: Detection configuration
    :param objects_cfg: Objects configuration
    :return: rob, map, number_gate_pillars
    """
    # Preset the variables
    both_seen = False
    rotation_cnt = 0

    # We assume that the robot is oriented towards the first pillar
    # Rotate the robot to the right two times
    for i in range(2):
        # Execute two small rotations
        small_rot_move.execute_small_rot_positive(5, 0.9)
        rotation_cnt += 1

        # Analyze the current situation
        map, number_gate_pillars, goal, _ = world_analysis(rob, detection_cfg, objects_cfg)

        # Stop if the mission is accomplished (two pillars are seen)
        if number_gate_pillars == 2:
            both_seen = True
            rotation_cnt -= 1
            break

    # If the robot has not seen two pillars, rotate back to the default position
    if number_gate_pillars != 2:
        for i in range(rotation_cnt):
            # Execute two small rotations (make the robot wait to finish the rotation)
            time.sleep(1)
            small_rot_move.execute_small_rot_negative(5, 0.9)

    # If the robot has not seen two pillars, rotate to the left two times
    rotation_cnt = 0
    if not both_seen:
        for i in range(2):
            # Execute two small rotations
            small_rot_move.execute_small_rot_negative(5, 0.9)
            rotation_cnt += 1

            # Analyze the current situation
            map, number_gate_pillars, goal, _ = world_analysis(rob, detection_cfg, objects_cfg)

            # Stop if the mission is accomplished (two pillars are seen)
            if number_gate_pillars == 2:
                rotation_cnt -= 1
                break

    # If the robot has not seen two pillars, rotate back to the default position
    if number_gate_pillars != 2:
        for i in range(rotation_cnt):
            # Execute two small rotations (make the robot wait to finish the rotation)
            time.sleep(1)
            small_rot_move.execute_small_rot_positive(5, 0.9)

    return rob, map, number_gate_pillars


def find_best_position_to_see_garage(rob, small_rot_move, map, number_gate_pillars, detection_cfg, objects_cfg):
    """
    When the robot sees the garage and does not see the gate, this function will rotate the robot to find the best
    position to see the garage. The number of garage points on the map is used as a metric.
    :param rob: Robot object
    :param small_rot_move: move object
    :param map: Map object
    :param number_gate_pillars: Number of the gate pillars
    :param detection_cfg: Detection configuration
    :param objects_cfg: Objects configuration
    :return: None
    """
    # Preset the variables
    max_val = -1
    seen = False

    # Rotate the robot until the best position is found
    while True:
        # If we can see gate, break from the loop
        if number_gate_pillars != 0:
            break

        # Get the number of the garage points on the map
        num_points = len(map.get_garage().get_world_coordinates()[0]) if map.get_garage() is not None else -1

        # If we cannot see any yellow point and we have already seen it, start the search for the best position by
        # rotating the robot back (stop when the number of points starts to decrease)
        if num_points == -1 and seen:
            max_val = 0
            num_points = 0
        elif num_points != -1 and not seen:
            seen = True

        if max_val == -1:
            small_rot_move.execute_small_rot_positive(2, 1)
        else:
            if num_points >= max_val:
                max_val = num_points
                small_rot_move.execute_small_rot_negative(2, 1)
            else:
                # The robot has over-rotated, rotate back to the best position and end the searching process
                small_rot_move.execute_small_rot_positive(2, 1)
                break
        map, number_gate_pillars, goal, _ = world_analysis(rob, detection_cfg, objects_cfg, fill_map=False)


def huge_test() -> None:
    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))

    dims = detection_cfg['map']['dimensions']
    res = detection_cfg['map']['resolution']

    map = Map(dims, res, detection_cfg)

    rad = objects_cfg['robot']['radius']
    hei = objects_cfg['robot']['height']
    col = objects_cfg['robot']['color']

    rob = Robot(rad, col, 'black')
    print('robot object created')
    print('bumper initialized')

    rob.set_world_coordinates((0, 0))
    map.set_robot(rob)

    img = rob.take_rgb_img()
    pc = rob.take_point_cloud()

    det = Detector(map, img, pc, detection_cfg, objects_cfg)
    det.process_rgb()
    det.process_point_cloud()

    map.fill_world_map()

    vis = Visualizer(img, pc, map, det.get_processed_rgb(), det.get_processed_point_cloud(), detection_cfg)

    search_algorithm = detection_cfg['map']['search_algorithm']

    path = map.find_way((250, 0), tuple(map.get_goal()), search_algorithm)

    vis.visualize_rgb()
    # vis.visualize_point_cloud()
    vis.visualize_map(path=path)

    tmp = move.Move(rob, path, detection_cfg)
    print(path)
    tmp.execute_move()


def big_test(img: np.ndarray, pc: np.ndarray) -> None:
    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))

    dims = detection_cfg['map']['dimensions']
    res = detection_cfg['map']['resolution']

    map = Map(dims, res, detection_cfg)

    # Set up robot -------------------------------
    rad = objects_cfg['robot']['radius']
    hei = objects_cfg['robot']['height']
    col = objects_cfg['robot']['color']

    rob = Robot(rad, col, 'black')
    print('robot object created')
    print('bumper initialized')

    rob.set_world_coordinates((0, 0))
    map.set_robot(rob)
    # --------------------------------------------

    det = Detector(map, img, pc, detection_cfg, objects_cfg)
    det.process_rgb()
    det.process_point_cloud()

    map.fill_world_map()
    search_algorithm = detection_cfg['map']['search_algorithm']
    
    path = map.find_way((250, 0), tuple(map.get_goal()), search_algorithm)

    gar_coord = map.get_garage().get_world_coordinates()
    gar_map_x = map.conv_real_to_map(gar_coord[0], True)
    gar_map_y = map.conv_real_to_map(gar_coord[1])

    gar_coord_map = np.array([gar_map_x, gar_map_y])

    # Save garage coordinates to file
    np.save("garage_coordinates.npy", gar_coord_map)

    vis = Visualizer(img, pc, map, det.get_processed_rgb(), det.get_processed_point_cloud(), detection_cfg)

    #vis.visualize_rgb()
    #vis.visualize_point_cloud()
    vis.visualize_map(path=path)
    #path = [(250, 0), (251, 1), (252, 2), (253, 3), (254, 4), (255, 5), (256, 6), (257, 7), (258, 8), (259, 9), (260, 10), (261, 11), (262, 12), (263, 13), (264, 14), (265, 15), (266, 16), (267, 17), (268, 18), (269, 19), (270, 20), (271, 21), (272, 22), (273, 23), (274, 24), (275, 25), (276, 26), (277, 27), (278, 28), (279, 29), (280, 30), (281, 31), (282, 32), (282, 33), (282, 34), (282, 35), (282, 36), (282, 37), (282, 38), (282, 39), (282, 40), (282, 41), (282, 42), (282, 43), (282, 44), (282, 45), (282, 46), (282, 47), (282, 48), (282, 49), (282, 50), (282, 51), (282, 52), (282, 53), (282, 54), (282, 55), (282, 56), (282, 57), (282, 58), (282, 59), (282, 60), (282, 61), (282, 62), (283, 63), (284, 64), (285, 65), (286, 66), (287, 67), (288, 68), (289, 69), (290, 70), (291, 71), (292, 72), (293, 73), (293, 74), (293, 75), (293, 76), (293, 77), (293, 78), (293, 79), (293, 80), (293, 81), (293, 82), (293, 83), (293, 84), (293, 85), (292, 86), (291, 87), (290, 88), (289, 89), (288, 90), (287, 91), (286, 92), (285, 93), (284, 94), (284, 95), (284, 96), (284, 97), (284, 98), (284, 99), (284, 100), (284, 101), (284, 102), (284, 103), (284, 104), (284, 105), (284, 106), (284, 107), (284, 108), (284, 109), (284, 110), (284, 111), (284, 112), (284, 113), (284, 114)]

    # tmp = move.move(rob, path)
    # print(path)
    # tmp.execute_move()


def image_man(img: np.ndarray, pc: np.ndarray) -> None:

    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))

    dims = detection_cfg['map']['dimensions']
    res = detection_cfg['map']['resolution']

    map = Map(dims, res, detection_cfg)

    det = Detector(map, img, pc, detection_cfg, objects_cfg)
    det.process_rgb()
    det.process_point_cloud()

    vis = Visualizer(img, pc, map, det.get_processed_rgb(), det.get_processed_point_cloud(), detection_cfg)
    vis.visualize_rgb()
    vis.visualize_point_cloud()


def map_visualization_test() -> None:
    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))

    # create a map
    dims = detection_cfg['map']['dimensions']
    res = detection_cfg['map']['resolution']
    map = Map(dims, res, detection_cfg)

    # create a garage
    len = objects_cfg['garage']['length']
    wid = objects_cfg['garage']['width']
    hei = objects_cfg['garage']['height']
    col = objects_cfg['garage']['color']

    garage = Garage(len, wid, hei, None, col)
    garage.set_world_coordinates((0, 3))
    garage.set_orientation(0)

    # create a gate
    wid = objects_cfg['gate']['pillars_width']
    hei = objects_cfg['gate']['pillars_height']
    pillars_dis = objects_cfg['gate']['pillars_distance']
    col = objects_cfg['gate']['color']

    gate = Gate(wid, hei, col, None, [1, 2], pillars_dis, None, (garage.get_length(), garage.get_width(), garage.get_height()))
    length = garage.get_length()
    p1 = (0, 3)
    angle = 120
    angle = np.deg2rad(angle)
    p2 = (p1[0] + np.cos(angle) * length, p1[1] + np.sin(angle) * length)
    gate.set_world_coordinates([p1, p2])
    gate.calculate_orientation()

    # create a robot
    rad = objects_cfg['robot']['radius']
    hei = objects_cfg['robot']['height']
    col = objects_cfg['robot']['color']

    robot = Robot(rad, col, 'black')
    robot.set_world_coordinates((0, 0))

    # create multiple obstacles
    rad = objects_cfg['obstacle']['radius']
    hei = objects_cfg['obstacle']['height']

    obstacle = Obstacle(rad, hei, 'red', None, None)
    obstacle.set_world_coordinates((-1, 1))

    obstacle2 = Obstacle(rad, hei, 'blue', None, None)
    obstacle2.set_world_coordinates((1, 1))

    obstacle3 = Obstacle(rad, hei, 'green', None, None)
    obstacle3.set_world_coordinates((0, 1))

    # add the objects to the map
    map.set_garage(garage)
    map.set_robot(robot)
    map.set_obstacle(obstacle)
    map.set_obstacle(obstacle2)
    map.set_obstacle(obstacle3)
    map.set_gate(gate)

    # visualize the map
    # map.visualize()

    # visualize the map in the world
    map.fill_world_map()

    # cv.rectangle(map.world_map, (0, 50), (400, 50 + 50), 4, -1)
    # cv.rectangle(map.world_map, (100, 200), (500, 200+50), 4, -1)

    path = map.find_way((0, 250), (280, 280))

    vis = Visualizer(None, None, map, None, detection_cfg)
    vis.visualize_map(path)

    return path


def main():
    test = "automate"
    if test == "image":
        for i in range(16):
            if i == 3:
                continue
            img = cv.imread(f'camera/shoot1/{i}.png')
            image_man(img, None)
            cv.waitKey(0)
            cv.destroyAllWindows()

        for i in range(9):
            img = cv.imread(f'camera/shoot1/{i}.png')
            image_man(img, None)
            cv.waitKey(0)
            cv.destroyAllWindows()
    elif test == "map":
        map_visualization_test()
    elif test == "pc":
        for i in range(15):
            img = cv.imread(f'camera/shoot3/RGB{i}.png')
            pc = np.load(f'camera/shoot3/PC{i}.npy')
            image_man(img, pc)
    elif test == "pc1":
        img = cv.imread(f'camera/shoot6/RGB0.png')
        pc = np.load(f'camera/shoot6/PC0.npy')
        image_man(img, pc)
    elif test == "big":
        all = False
        if all:
            for i in range(15):
                img = cv.imread(f'camera/shoot3/RGB{i}.png')
                pc = np.load(f'camera/shoot3/PC{i}.npy')
                big_test(img, pc)
        else:
            img = cv.imread(f'camera/shoot7/RGB0.png')
            pc = np.load(f'camera/shoot7/PC0.npy')
            big_test(img, pc)
    elif test == "huge":
        huge_test()
    elif test == "automate":
        automate_test()


if __name__ == '__main__':
    main()

