import numpy as np
import math
import time

from .robot import Robot
from .move import Move
from .regulated_move import RegulatedMove
from .map import Map
from .visualizer import Visualizer
from .detector import Detector


# SEQUENCE FUNCTIONS
def park(rob, detection_cfg, objects_cfg, move_cfg) -> None:
    """
    Function that parks the robot when it is in front of the gate.
    :param rob: Robot object
    :param detection_cfg: Configuration file for detection
    :param objects_cfg: Configuration file for objects
    :param move_cfg: Configuration file for movement
    :return: None
    """
    # Create RegulatedMove object
    move = RegulatedMove(rob, move_cfg)

    # Analyze the situation
    map, number_gate_pillars, goal, path = world_analysis(rob, detection_cfg, objects_cfg, fill_map=False)

    # Orient the robot towards the garage
    find_best_position_to_see_garage(rob, move, map, number_gate_pillars, detection_cfg, objects_cfg,
                                     parking=True)

    # Analyze the situation
    time.sleep(0.5)
    map, _, goal, path = world_analysis(rob, detection_cfg, objects_cfg, visualize=False, fill_map=False)

    # Get garage sides
    garage_sides = get_garage_sides(map)

    # Check if the garage sides were found
    if garage_sides is None or len(garage_sides) == 0:
        print("No garage sides found! Try again...")
        angle = np.random.randint(5, 20)
        move.execute_small_rot_negative(angle, 0.5)
        park(rob, detection_cfg, objects_cfg)
    elif len(garage_sides) == 1:
        print("Only one garage side found! Try again...")
        angle = np.random.randint(5, 20)
        move.execute_small_rot_negative(angle, 0.5)
        park(rob, detection_cfg, objects_cfg)
    else:
        # Get a unit vector for each garage side
        garage_sides_unit_vectors = []
        garage_sides_points = []
        for side in garage_sides:
            a, b = side
            if a > 0:
                v = np.array([1, a])
            elif a < 0:
                v = np.array([-1, -a])
            else:
                v = np.array([0, 1])

            # Make it a unit vector
            v_unit = v / np.linalg.norm(v)
            garage_sides_unit_vectors.append(v_unit)

            # Get one point on each garage side (does not matter which one)
            garage_sides_points.append([0, b])

        # Get the angle between the two garage sides
        angle = np.arccos(np.dot(garage_sides_unit_vectors[0], garage_sides_unit_vectors[1]) / (
                np.linalg.norm(garage_sides_unit_vectors[0]) * np.linalg.norm(garage_sides_unit_vectors[1])))

        angle = np.degrees(angle)

        # Check if the angle is around 90 degrees (if so try to fit it again)
        if abs(angle - 90) > detection_cfg['perpendicular_angle_threshold']:
            print("Fit error is too large! Try again...")
            angle = np.random.randint(5, 20)
            move.execute_small_rot_negative(angle, 0.5)
            park(rob, detection_cfg, objects_cfg)

        # Get the intersection point of the two garage sides and make its coordinates integers
        intersection_point = find_intersection_point(garage_sides_points[0], garage_sides_unit_vectors[0],
                                                     garage_sides_points[1], garage_sides_unit_vectors[1])
        intersection_point = np.array(intersection_point).astype(np.int32)

        # Get garage dimensions
        garage_length = map.conv_real_to_map(objects_cfg['garage']['length'])
        garage_width = map.conv_real_to_map(objects_cfg['garage']['width'])

        # Decide which side is the back side of the garage (it should usually be the first one)
        back_side_idx = 0 if abs(garage_sides[0][0]) < abs(garage_sides[1][0]) else 1
        front_side_idx = 1 - back_side_idx

        # Get the point in the middle of the garage and make its coordinates integers
        middle_point = intersection_point - garage_sides_unit_vectors[back_side_idx] * garage_length / 2.5 \
                        - garage_sides_unit_vectors[front_side_idx] * garage_width / 2.5
        middle_point = middle_point.astype(np.int32)

        # Get the robot's position
        robot_pos = detection_cfg['map']['start_point']

        # Get the center point in front of the garage
        mid_front_point = find_closest_point_on_line(middle_point, garage_sides_unit_vectors[front_side_idx], robot_pos)

        # Check if the point is inside the map (if not, correct it)
        if mid_front_point[0] < 0:
            mid_front_point[0] = 0
        mid_front_point = np.array(mid_front_point).astype(np.int32)

        # Create a path of points that the robot will follow from robot position to the closest point on the center line
        search_algorithm = detection_cfg["map"]["search_algorithm"]
        path = map.find_way(robot_pos, mid_front_point, search_algorithm)

        # The second part of the path has to be interpolated (we do not need the optimal path, just a straight line)
        # -> take a vector from the middle point to the middle point in front of the garage and gradually scale it
        front_to_middle_vector = middle_point - mid_front_point
        inter_points = detection_cfg["interpolation_points"]
        for i in range(inter_points):
            start = mid_front_point + front_to_middle_vector * i / inter_points
            start = np.array(start).astype(np.int32)
            end = mid_front_point + front_to_middle_vector * (i + 1) / inter_points
            end = np.array(end).astype(np.int32)
            path += map.find_way(start, end, search_algorithm)

        # Follow the path
        move.go(path)


def get_to_gate(rob, detection_cfg, objects_cfg, move_cfg) -> None:
    """
    Function that gets the robot to the gate.
    :param rob: Robot object
    :param detection_cfg: Configuration file for detection
    :param objects_cfg: Configuration file for objects
    :param move_cfg: Configuration file for movement
    :return: None
    """
    # Create RegulatedMove object
    move = RegulatedMove(rob, move_cfg)

    # STATE AUTOMAT
    while True:
        # Extract information from the surrounding world
        map, number_gate_pillars, goal, _ = world_analysis(rob, detection_cfg, objects_cfg)

        # If no goal is set (robot does not see the garage [yellow] nor the gate [magenta]) then rotate and search
        if goal is None:
            # Rotate the robot
            move.execute_small_rot_positive(20, 0.9)
            continue

        # We have found the garage/gate, try to improve the robot's position to make the goal easier to set
        # If the goal is set then there are two options:
        else:
            # The robot sees one pillar of the gate
            if number_gate_pillars == 1:
                # Try to find the second pillar
                find_more_pillars(rob, move, map, number_gate_pillars, detection_cfg, objects_cfg)
            # The robot sees the garage but not the gate
            elif number_gate_pillars == 0:
                # Try to find the best position to see the gate
                find_best_position_to_see_garage(rob, move, map, number_gate_pillars, detection_cfg,
                                                 objects_cfg)
        # END OF THE STATE AUTOMAT
        # -> find the best path to the goal and follow it

        # Analyze the world and find the best path and wait for the robot to stop
        time.sleep(0.5)
        map, number_gate_pillars, goal, path = world_analysis(rob, detection_cfg, objects_cfg, visualize=False)

        # Follow the path
        move.go(path)

        if not rob.get_stop():
            if map.get_goal_type() == detection_cfg['map']['goal_type']['two_pillars'] or \
                    map.get_goal_type() == detection_cfg['map']['goal_type']['one_pillar']:
                # All conditions are met, we can start the parking sequence
                if found_gate(rob, detection_cfg, objects_cfg, move):
                    break
            elif map.get_goal_type() == detection_cfg['map']['goal_type']['garage'] and found_gate(rob, detection_cfg,
                                                                                                   objects_cfg,
                                                                                                   move):
                # All conditions are met, we can start the parking sequence
                if found_gate(rob, detection_cfg, objects_cfg, move):
                    break
        else:
            # Robot has stopped, we need to find the path again (and reset stop flag)
            rob.set_stop(False)


# HELPER FUNCTIONS
def world_analysis(rob, detection_cfg, objects_cfg, visualize=False, fill_map=True) -> (Map, int, tuple, list):
    """
    Function that takes image and point cloud from the robot and extracts information about the surrounding world.
    :param rob: Robot object
    :param detection_cfg: Configuration file for detection
    :param objects_cfg: Configuration file for objects
    :param visualize: Boolean value that determines if the process should be visualized
    :param fill_map: Boolean value that determines if the map should be filled with information
    :return: The map, number of pillars of the gate, goal object and path to the goal
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

    # Get goal and initialize path
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
            time.sleep(2)
            vis.visualize_map(path=path)
        else:
            vis.visualize_map()

    return map, number_gate_pillars, goal, path


def find_more_pillars(rob, move, map, number_gate_pillars, detection_cfg, objects_cfg) -> (Robot, Map, int):
    """
    When the robot sees one gate pillar, there is a possibility that it will see the second one if it rotates.
    This function will rotate the robot to find the second gate pillar.
    :param rob: Robot object
    :param move: RegulatedMove object
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
        # Execute two small rotations(make the robot wait to finish the rotation)
        time.sleep(2)
        move.execute_small_rot_positive(5, 0.9)
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
            time.sleep(2)
            move.execute_small_rot_negative(5, 0.9)

    # If the robot has not seen two pillars, rotate to the left two times
    rotation_cnt = 0
    if not both_seen:
        for i in range(2):
            # Execute two small rotations (make the robot wait to finish the rotation)
            time.sleep(2)
            move.execute_small_rot_negative(5, 0.9)
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
            time.sleep(2)
            move.execute_small_rot_positive(5, 0.9)

    return rob, map, number_gate_pillars


def find_best_position_to_see_garage(rob, move, map, number_gate_pillars, detection_cfg, objects_cfg,
                                     parking=False) -> None:
    """
    When the robot sees the garage and does not see the gate, this function will rotate the robot to find the best
    position to see the garage. The number of garage points on the map is used as a metric.
    :param rob: Robot object
    :param RegulatedMove: move object
    :param map: Map object
    :param number_gate_pillars: Number of the gate pillars
    :param detection_cfg: Detection configuration
    :param objects_cfg: Objects configuration
    :param parking: If the robot is parking
    :return: None
    """
    # Preset the variables
    max_val = -1
    seen = False

    # Rotate the robot until the best position is found
    while True:
        # If we can see gate, break from the loop
        if number_gate_pillars != 0 and not parking:
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
            move.execute_small_rot_positive(2, 1)
        else:
            if num_points >= max_val:
                max_val = num_points
                move.execute_small_rot_negative(2, 1)
            else:
                # The robot has over-rotated, rotate back to the best position and end the searching process
                move.execute_small_rot_positive(2, 1)
                break
        map, number_gate_pillars, goal, _ = world_analysis(rob, detection_cfg, objects_cfg, fill_map=False)


def rotate_vector(vector, angle) -> list:
    """
    Rotate a vector by a given angle
    :param vector: Vector to rotate
    :param angle: Angle to rotate by
    :return: Rotated vector
    """
    x = vector[0] * math.cos(angle) - vector[1] * math.sin(angle)
    y = vector[0] * math.sin(angle) + vector[1] * math.cos(angle)
    return [x, y]


def search_for_pillar(side, angle, move, map, rob, detection_cfg, objects_cfg) -> tuple:
    """
    This function will rotate the robot to find the second gate pillar.
    :param side: Side of the first pillar to search for the second one
    :param angle: Angle to rotate by
    :param RegulatedMove: move object
    :param map: Map object
    :param rob: Robot object
    :param detection_cfg: Detection configuration
    :param objects_cfg: Objects configuration
    :return: pillar1, pillar2 (each could be None)
    """
    # Set the first pillar
    pillar1 = map.get_gate().get_world_coordinates()[0]
    pillar2 = None

    # Turn to the side and search for the second pillar
    if side == "right":
        move.execute_small_rot_positive(angle, 0.5)
        angle = -angle
    else:
        move.execute_small_rot_negative(angle, 0.5)

    # Analyze the current situation
    time.sleep(0.5)
    map, number_gate_pillars, _, _ = world_analysis(rob, detection_cfg, objects_cfg, fill_map=False)

    # Check if we see at least one pillar
    if number_gate_pillars == 2:
        pillar1 = map.get_gate().get_world_coordinates()[0]
        pillar2 = map.get_gate().get_world_coordinates()[1]
        print('We found both during search_for_pillar()!')
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
    Find a point on the line that is the closest to the given point. The line is defined by a point and a vector.
    :param line_point: Point on the line
    :param line_vector: Vector of the line
    :param point: Point to find the closest point on the line to
    :return: Closest point on the line [x, y]
    """
    # Get a vector perpendicular to the line
    perp_vector = [line_vector[1], -line_vector[0]]

    # Calculate the intersection point of the line and the perpendicular line going through the point
    intersection_point = find_intersection_point(line_point, line_vector, point, perp_vector)

    return intersection_point


def find_intersection_point(point1, vector1, point2, vector2) -> list:
    """
    Find the intersection point of two lines. The lines are defined by a point and a vector.
    :param point1: Point on the first line
    :param vector1: Vector of the first line
    :param point2: Point on the second line
    :param vector2: Vector of the second line
    :return: Intersection point [x, y]
    """
    # Calculate the determinant of the matrix
    det = vector1[0] * vector2[1] - vector1[1] * vector2[0]

    # If the determinant is zero, the lines are parallel
    if det == 0:
        return []

    # Calculate the intersection point
    t = (vector2[0] * (point1[1] - point2[1]) - vector2[1] * (point1[0] - point2[0])) / det
    intersection_point = [point1[0] + t * vector1[0], point1[1] + t * vector1[1]]

    return intersection_point


def found_gate(rob, detection_cfg, objects_cfg, move) -> bool:
    """
    Check if the robot sees the gate.
    :param rob: Robot object
    :param detection_cfg: Detection configuration
    :param objects_cfg: Objects configuration
    :param RegulatedMove: move object
    :return: True if the robot sees the gate, False otherwise
    """
    for i in range(10):
        time.sleep(1)
        move.execute_small_rot_positive(30, 1)
        map, number_gate_pillars, goal, _ = world_analysis(rob, detection_cfg, objects_cfg, fill_map=False)

        if number_gate_pillars != 0:
            return True
    return False


def get_garage_sides(map):
    """
    Return the fitted sides of the garage (RANSAC). The sides are defined by (a, b) where y = ax + b.
    :param map: Map object
    :return: The fitted sides of the garage
    """
    # Get the detected points of the garage
    points = map.garage.get_world_coordinates()

    # Convert real world parameters to map parameters
    xs = map.conv_real_to_map(points[0], add=True)
    ys = map.conv_real_to_map(points[1])

    # Fill the map with the detected points
    map.fill_in_garage([])

    # Fit the lines (it is sufficient to fit only two sides)
    lines = []
    for i in range(2):
        # Fit a line to the points
        xs, ys, line, inliers = map.fit_line(xs, ys)
        if line is not None:
            lines.append(line)
    return lines
