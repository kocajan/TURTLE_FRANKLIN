import math
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
    # Prepare variables
    pillar1 = None
    pillar2 = None

    # Create small rotation move object (used for small rotations during the searching process)
    small_rot_move = Move(rob, None, None)

    # Then search for both of the pillars (we will probably see only one of them at a time)
    while True:
        # Turn to the left and search for the pillars
        print("Searching for the pillars...")
        small_rot_move.execute_small_rot_positive(10, 0.5)
        map, number_gate_pillars = parking_analysis(rob, detection_cfg, objects_cfg)

        # Check if we see at least one pillar
        if number_gate_pillars != 0:
            print("Found at least one pillar!")
            break

    # If we have one pillar, then we need to find the second one
    if number_gate_pillars == 1:
        # Set angle
        angle = 20
        print(f"Found one pillar, searching for the second one... (angle = {angle})")

        # Get the first pillar and compensate for the angle
        pillar1 = map.get_gate().get_world_coordinates()[0]
        print("First pillar: ", pillar1)
        pillar1 = rotate_vector(pillar1, -angle)
        print("First pillar (rotated): ", pillar1)

        # Turn to the right and search for the second pillar
        print("Turning to the right...")
        tmp_pillar, pillar2 = search_for_pillar("right", angle, small_rot_move, map, rob, detection_cfg, objects_cfg)

        # If we haven't found the second pillar, so we need to turn back to the first one and turn to the left
        if pillar2 is None:
            print("The second pillar was not found, turning back to the first one...")
            # Turn back to the first pillar
            small_rot_move.execute_small_rot_positive(angle, 0.5)

            # Just to be sure, analyze the world again
            map, number_gate_pillars = parking_analysis(rob, detection_cfg, objects_cfg)

            # Get the first pillar and compensate for the angle
            pillar1 = map.get_gate().get_world_coordinates()[0]
            print("Redefined first pillar: ", pillar1)
            pillar1 = rotate_vector(pillar1, -angle)
            print("Redefined first pillar (rotated): ", pillar1)

            print("Turning to the left...")
            # Turn to the left and search for the second pillar
            tmp_pillar, pillar2 = search_for_pillar("left", angle, small_rot_move, map, rob, detection_cfg, objects_cfg)

            # If we haven't found the second pillar, then we are v piči
            if pillar2 is None:
                # TODO:
                print("We are v piči. They stole our precious.")
                exit(-1)

        print("Found the second pillar!")
        # We have found both pillars
        # If they are both from the same picture, we can find their map coordinates
        if tmp_pillar is not None:
            print("Both pillars are from the same picture, we can find their map coordinates.")
            pillar1 = tmp_pillar
    elif number_gate_pillars == 2:
        # Get the pillars
        pillar1 = map.get_gate().get_world_coordinates()[0]
        pillar2 = map.get_gate().get_world_coordinates()[1]
        print("Found both pillars!")
        print("First pillar: ", pillar1)
        print("Second pillar: ", pillar2)

    # We have now found both pillars, we can get map coordinates of the gate
    pillar1_map = (map.conv_real_to_map(pillar1[0], add=True), map.conv_real_to_map(pillar1[1]))
    pillar2_map = (map.conv_real_to_map(pillar2[0], add=True), map.conv_real_to_map(pillar2[1]))

    print("First pillar (map): ", pillar1_map)
    print("Second pillar (map): ", pillar2_map)

    # Get the center of the gate
    gate_center_map = ((pillar1_map[0] + pillar2_map[0]) // 2, (pillar1_map[1] + pillar2_map[1]) // 2)
    print("Gate center (map): ", gate_center_map)

    # Get perpendicular vector to the gate
    perpendicular_vector = (pillar2_map[1] - pillar1_map[1], pillar1_map[0] - pillar2_map[0])

    # Get the robot's position
    robot_pos = (map.conv_real_to_map(rob.get_world_coordinates()[0], add=True),
                 map.conv_real_to_map(rob.get_world_coordinates()[1]))

    # Find the closest point on a line leading through the gate's center and perpendicular to the gate
    closest_point = find_closest_point_on_line(gate_center_map, perpendicular_vector, robot_pos)

    # Convert the closest point to whole numbers (pixels)
    closest_point = (int(closest_point[0]), int(closest_point[1]))

    # Get a point on the line going through the gate's center and perpendicular to the gate
    # but on the other side of the gate than the robot
    # TODO

    # Create a path of points that the robot will follow from robot position to the closest point on the line
    search_algorithm = detection_cfg["map"]["search_algorithm"]
    print("start point: ", robot_pos, "\nend point: ", closest_point)
    path = map.find_way(robot_pos, closest_point, search_algorithm)
    print('path: ', path)

    # TODO: delte this
    # Visualize the situation (path, points, ...)
    import numpy as np
    map = np.zeros_like(map.get_world_map())

    # Fill the map
    for point in path:
        map[point[0], point[1]] = 1

    # Draw a circle around the points
    import cv2
    map = cv2.circle(map, robot_pos, 5, 0, -1)
    map = cv2.circle(map, closest_point, 5, 2, -1)
    map = cv2.circle(map, gate_center_map, 5, 3, -1)
    map = cv2.circle(map, pillar1_map, 5, 4, -1)
    map = cv2.circle(map, pillar2_map, 5, 4, -1)

    # Show the map (color use colormap to distinguish between points - not grey)
    import matplotlib.pyplot as plt
    plt.imshow(map, cmap='tab10')
    plt.show()

    # Execute path
    tmp = Move(rob, path, detection_cfg)
    tmp.execute_move()


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
    for _ in range(2):
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
        for _ in range(rotation_cnt):
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
        for _ in range(rotation_cnt):
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


def rotate_vector(vector, angle):
    """
    Rotate a vector by a given angle
    :param vector: Vector to rotate
    :param angle: Angle to rotate by
    :return: Rotated vector
    """
    x = vector[0] * math.cos(angle) - vector[1] * math.sin(angle)
    y = vector[0] * math.sin(angle) + vector[1] * math.cos(angle)
    return [x, y]


def search_for_pillar(side, angle, small_rot_move, map, rob, detection_cfg, objects_cfg) -> tuple:
    """
    This function will rotate the robot to find the second gate pillar.
    :param side: Side of the first pillar to search for the second one
    :param angle: Angle to rotate by
    :param small_rot_move: move object
    :param map: Map object
    :param rob: Robot object
    :param detection_cfg: Detection configuration
    :param objects_cfg: Objects configuration
    :return: None
    """
    # Set the first pillar
    pillar1 = map.get_gate().get_world_coordinates()[0]
    pillar2 = None

    # Turn to the side and search for the second pillar
    if side == "left":
        small_rot_move.execute_small_rot_positive(angle, 0.5)
    else:
        small_rot_move.execute_small_rot_negative(angle, 0.5)
        angle = -angle

    # Analyze the current situation
    map, number_gate_pillars = parking_analysis(rob, detection_cfg, objects_cfg)

    # Check if we see at least one pillar
    if number_gate_pillars == 2:
        pillar1 = map.get_gate().get_world_coordinates()[0]
        pillar2 = map.get_gate().get_world_coordinates()[1]
        return pillar1, pillar2
    elif number_gate_pillars == 1:
        # Check if the pillar is not the same as the first one (it is not in predefined radius)
        pillar2 = map.get_gate().get_world_coordinates()[0]

        # Rotate coordinates of the first pillar
        pillar1 = rotate_vector(pillar1, -angle)

        # Calculate the distance
        distance = ((pillar1[0] - pillar2[0]) ** 2 + (pillar1[1] - pillar2[1]) ** 2) ** 0.5

        if distance < 5:
            pillar2 = None
    return None, pillar2


def find_closest_point_on_line(line_point, line_vector, point) -> list:
    """
    Find the closest point on the line to the given point. The line is defined by a point and a vector.
    """
    # Get a vector perpendicular to the line
    perp_vector = [line_vector[1], -line_vector[0]]

    # Calculate the intersection point of the line and the perpendicular line going through the point
    intersection_point = find_intersection_point(line_point, line_vector, point, perp_vector)

    return intersection_point


def find_intersection_point(point1, vector1, point2, vector2) -> list:
    """
    Find the intersection point of two lines. The lines are defined by a point and a vector.
    """
    # Calculate the determinant of the matrix
    det = vector1[0] * vector2[1] - vector1[1] * vector2[0]

    # If the determinant is zero, the lines are parallel
    if det == 0:
        return []

    # Calculate the intersection point
    t = (vector2[0] * (point1[1] - point2[1]) - vector2[1] * (point1[0] - point2[0])) / det
    intersection_point = [point1[0] + t * vector1[0], point1[1] + t * vector1[1]]

    # TODO: delete this
    # Visualize the situation
    import matplotlib.pyplot as plt
    plt.plot([point1[0], point1[0] + vector1[0]], [point1[1], point1[1] + vector1[1]], 'b')
    plt.plot([point2[0], point2[0] + vector2[0]], [point2[1], point2[1] + vector2[1]], 'r')
    plt.plot(intersection_point[0], intersection_point[1], 'go', markersize=10)
    plt.show()


    return intersection_point

