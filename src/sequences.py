import time

from .robot import Robot
from .move import Move
from .map import Map
from .visualizer import Visualizer
from .detector import Detector


# SEQUENCE FUNCTIONS
def park(rob, detection_cfg, objects_cfg) -> None:
    """
    Function that parks the robot when it is in front of the gate.
    :param rob: Robot object
    :param detection_cfg: Configuration file for detection
    :param objects_cfg: Configuration file for objects
    :return: None
    """
    # Create small rotation move object (used for small rotations during the searching process)
    small_rot_move = Move(rob, None, None)

    # Then search for both of the pillars (we will probably see only one of them at a time)
    for i in range(18):
        small_rot_move.execute_small_rot_positive(5, 0.5)
    # while True:
        # Turn to the left and search for the pillars
        # small_rot_move.execute_small_rot_positive(5, 0.5)
        # map, number_gate_pillars = parking_analysis(rob, detection_cfg, objects_cfg)


def get_to_gate(rob, detection_cfg, objects_cfg) -> None:
    """
    Function that gets the robot to the gate.
    :param rob: Robot object
    :param detection_cfg: Configuration file for detection
    :param objects_cfg: Configuration file for objects
    :return: None
    """
    # Create small rotation move object (used for small rotations during the searching process)
    small_rot_move = Move(rob, None, None)

    # STATE AUTOMAT
    while True:
        # Extract information from the surrounding world
        map, number_gate_pillars, goal, _ = world_analysis(rob, detection_cfg, objects_cfg)

        # If no goal is set (robot does not see the garage [yellow] nor the gate [magenta]) then rotate and search
        if goal is None:
            # Rotate the robot
            small_rot_move.execute_small_rot_positive(20, 0.9)
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
        map, number_gate_pillars, goal, path = world_analysis(rob, detection_cfg, objects_cfg, visualize=True)

        # Follow the path
        tmp = Move(rob, path, detection_cfg)
        tmp.execute_move()

        if not rob.get_stop():
            if map.get_goal_type() == detection_cfg['map']['goal_type']['two_pillars'] or \
                map.get_goal_type() == detection_cfg['map']['goal_type']['one_pillar']:
                # All conditions are met, we can start the parking sequence
                break
        else:
            # Robot has stopped, we need to find the path again (and reset stop flag)
            rob.set_stop(False)


# HELPER FUNCTIONS
def parking_analysis(rob, detection_cfg, objects_cfg) -> (Map, int):
    """
    Function that takes image and point cloud from the robot and extracts information about the surrounding world
    during the parking sequence.
    :param rob: Robot object
    :param detection_cfg: Configuration file for detection
    :param objects_cfg: Configuration file for objects
    :return: The map, number of pillars of the gate
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

    # Get information to return
    gate = map.get_gate()
    if gate is not None:
        number_gate_pillars = gate.get_num_pillars()
    else:
        number_gate_pillars = 0

    return map, number_gate_pillars


def world_analysis(rob, detection_cfg, objects_cfg, visualize=False, fill_map=True) -> (Map, int, tuple, list):
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


def find_more_pillars(rob, small_rot_move, map, number_gate_pillars, detection_cfg, objects_cfg) -> (Robot, Map, int):
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


def find_best_position_to_see_garage(rob, small_rot_move, map, number_gate_pillars, detection_cfg, objects_cfg) -> None:
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
