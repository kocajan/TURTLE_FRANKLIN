import numpy as np
import math
import time

from .robot import Robot
from .move import Move
from .regulated_move import regulated_move
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

    # Analyze the situation
    #map, number_gate_pillars, goal, path = world_analysis(rob, detection_cfg, objects_cfg, fill_map=False)

    # Orient the robot towards the garage
    #find_best_position_to_see_garage(rob, small_rot_move, map, number_gate_pillars, detection_cfg, objects_cfg,
                                     #parking=True)

    # Analyze the situation
    #time.sleep(0.5)
    map, _, goal, path = world_analysis(rob, detection_cfg, objects_cfg, visualize=False, fill_map=False)

    # Get garage sides
    garage_sides, world_map = get_garage_sides(rob, map, detection_cfg, objects_cfg)

    if garage_sides is None or len(garage_sides) == 0:
        print("No garage sides found! Try again...")
        angle = np.random.randint(5, 20)
        small_rot_move.execute_small_rot_negative(angle, 0.5)
        park(rob, detection_cfg, objects_cfg)
    elif len(garage_sides) == 1:
        print("Only one garage side found! Try again...")
        angle = np.random.randint(5, 20)
        small_rot_move.execute_small_rot_negative(angle, 0.5)
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
        print("Angle between the two garage sides: ", angle)

        # Check if the angle is around 90 degrees (if so try to fit it again)
        if abs(angle - 90) > detection_cfg['perpendicular_angle_threshold']:
            print("Fit error is too large! Try again...")
            angle = np.random.randint(5, 20)
            small_rot_move.execute_small_rot_negative(angle, 0.5)
            park(rob, detection_cfg, objects_cfg)

        # Get the intersection point of the two garage sides
        intersection_point = find_intersection_point(garage_sides_points[0], garage_sides_unit_vectors[0],
                                                     garage_sides_points[1], garage_sides_unit_vectors[1])
        intersection_point = np.array(intersection_point).astype(np.int32)

        # Get garage dimensions
        garage_length = map.conv_real_to_map(objects_cfg['garage']['length'])
        garage_width = map.conv_real_to_map(objects_cfg['garage']['width'])

        # Decide which side is the back side of the garage (it should usually be the first one)
        back_side_idx = 0 if abs(garage_sides[0][0]) < abs(garage_sides[1][0]) else 1
        front_side_idx = 1 - back_side_idx

        # Get the point in the middle of the garage
        middle_point = intersection_point + garage_sides_unit_vectors[back_side_idx] * garage_length / 2 \
                       + garage_sides_unit_vectors[front_side_idx] * garage_width / 2
        middle_point = middle_point.astype(np.int32)

        print("Middle point: ", middle_point)

        # Show the lines on the map (the lines are in format of (a, b) where y = ax + b)
        world_map = map.get_world_map()
        import cv2 as cv
        # Generate points on the lines and draw them on the map as circles using numpy
        X = np.arange(0, world_map.shape[1])
        for line in garage_sides:
            a, b = line
            Y = a * X + b
            points = np.array([X, Y]).T

            # Take only valid points
            points = points[(points[:, 1] >= 0) & (points[:, 1] < world_map.shape[0])]
            points = points.astype(np.int32)

            for point in points:
                world_map[point[1], point[0]] = 1

        world_map[intersection_point[1], intersection_point[0]] = 2
        world_map[middle_point[1], middle_point[0]] = 3

        import matplotlib.pyplot as plt
        from matplotlib.colors import ListedColormap

        color_map = ListedColormap(["white", "black", "green", "pink", "yellow", "magenta", "red", "blue", "grey",
                                    "silver"])
        color_map = [color_map]

        n = len(color_map)
        fig, axs = plt.subplots(1, n, figsize=(10, 10), constrained_layout=True, squeeze=False)
        for [ax, cmap] in zip(axs.flat, color_map):
            psm = ax.pcolormesh(world_map, cmap=cmap, rasterized=True, vmin=0, vmax=detection_cfg["map"]["max_id"])
            fig.colorbar(psm, ax=ax)
        plt.show()


def park1(rob, detection_cfg, objects_cfg) -> None:
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
        map, number_gate_pillars = parking_analysis(rob, detection_cfg, objects_cfg)

        # Check if we see at least one pillar
        if number_gate_pillars != 0:
            print("Found at least one pillar!")
            break
        small_rot_move.execute_small_rot_positive(10, 0.5)

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
            small_rot_move.execute_small_rot_negative(angle, 0.5)

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

    # Get perpendicular vector to the gate (normalized)
    perpendicular_vector = (pillar2_map[1] - pillar1_map[1], pillar1_map[0] - pillar2_map[0])
    perpendicular_vector = np.array(perpendicular_vector)
    perpendicular_vector = perpendicular_vector / np.linalg.norm(perpendicular_vector)

    # Get the robot's position
    robot_pos = (map.conv_real_to_map(rob.get_world_coordinates()[0], add=True),
                 map.conv_real_to_map(rob.get_world_coordinates()[1]))

    # Find the closest point on a line leading through the gate's center and perpendicular to the gate
    closest_point = find_closest_point_on_line(gate_center_map, perpendicular_vector, robot_pos)

    # Convert the closest point to whole numbers (pixels)
    closest_point = (int(closest_point[0]), int(closest_point[1]))

    # Get a point on the line going through the gate's center and perpendicular to the gate that is on the other side
    # of the gate than the robot
    distance_from_gate = np.linalg.norm(np.array(pillar1) - np.array(pillar2))
    final_point1 = (gate_center_map[0] + perpendicular_vector[0] * distance_from_gate,
                    gate_center_map[1] + perpendicular_vector[1] * distance_from_gate)
    final_point2 = (gate_center_map[0] - perpendicular_vector[0] * distance_from_gate,
                    gate_center_map[1] - perpendicular_vector[1] * distance_from_gate)

    # Choose the one with greater y coordinate
    final_point = final_point1 if final_point1[1] > final_point2[1] else final_point2

    # Convert the final point to whole numbers (pixels)
    final_point = (int(final_point[0]), int(final_point[1]))

    # Create a path of points that the robot will follow from robot position to the closest point on the line
    search_algorithm = detection_cfg["map"]["search_algorithm"]
    print("start point: ", robot_pos, "\nend point: ", closest_point)
    path1 = map.find_way(robot_pos, closest_point, search_algorithm)
    path2 = map.find_way(closest_point, final_point, search_algorithm)
    path = path1 + path2
    print('path: ', path)

    # TODO: delete this
    # Visualize the situation (path, points, ...)
    # Show the map
    import matplotlib.pyplot as plt
    plt.scatter(robot_pos[0], robot_pos[1], color='red', label='robot position', s=20)
    plt.scatter(closest_point[0], closest_point[1], color='green', label='closest point', s=10)
    plt.scatter(gate_center_map[0], gate_center_map[1], color='blue', label='gate center', s=10)
    plt.scatter(pillar1_map[0], pillar1_map[1], color='yellow', label='pillar1', s=10)
    plt.scatter(pillar2_map[0], pillar2_map[1], color='orange', label='pillar2', s=10)
    plt.scatter(np.array(path)[:, 0], np.array(path)[:, 1], color='black', label='path', s=1)
    plt.xlim(0, 500)
    plt.ylim(0, 500)
    plt.legend()
    plt.show()

    # Execute path
    # tmp = Move(rob, path, detection_cfg)
    # tmp.execute_move()
    tmp = regulated_move(rob)
    tmp.go(path)


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
        # tmp = Move(rob, path, detection_cfg)
        # tmp.execute_move()
        tmp = regulated_move(rob)
        tmp.go(path)

        if not rob.get_stop():
            if map.get_goal_type() == detection_cfg['map']['goal_type']['two_pillars'] or \
                    map.get_goal_type() == detection_cfg['map']['goal_type']['one_pillar']:
                print(map.get_goal_type())
                # All conditions are met, we can start the parking sequence
                if found_gate(rob, detection_cfg, objects_cfg, small_rot_move):
                    break
            elif map.get_goal_type() == detection_cfg['map']['goal_type']['garage'] and found_gate(rob, detection_cfg,
                                                                                                   objects_cfg,
                                                                                                   small_rot_move):
                # All conditions are met, we can start the parking sequence
                if found_gate(rob, detection_cfg, objects_cfg, small_rot_move):
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
    # Wait for the robot ot fully stop
    time.sleep(0.5)

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
            time.sleep(2)
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
        # Execute two small rotations(make the robot wait to finish the rotation)
        time.sleep(2)
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
            time.sleep(2)
            small_rot_move.execute_small_rot_negative(5, 0.9)

    # If the robot has not seen two pillars, rotate to the left two times
    rotation_cnt = 0
    if not both_seen:
        for i in range(2):
            # Execute two small rotations (make the robot wait to finish the rotation)
            time.sleep(2)
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
            time.sleep(2)
            small_rot_move.execute_small_rot_positive(5, 0.9)

    return rob, map, number_gate_pillars


def find_best_position_to_see_garage(rob, small_rot_move, map, number_gate_pillars, detection_cfg, objects_cfg,
                                     parking=False) -> None:
    """
    When the robot sees the garage and does not see the gate, this function will rotate the robot to find the best
    position to see the garage. The number of garage points on the map is used as a metric.
    :param rob: Robot object
    :param small_rot_move: move object
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
    if side == "right":
        small_rot_move.execute_small_rot_positive(angle, 0.5)
        angle = -angle
    else:
        small_rot_move.execute_small_rot_negative(angle, 0.5)

    # Analyze the current situation
    map, number_gate_pillars = parking_analysis(rob, detection_cfg, objects_cfg)

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

    return intersection_point


def found_gate(rob, detection_cfg, objects_cfg, small_rotation) -> bool:
    """
    Check if the robot sees the gate.
    :param rob: Robot object
    :param detection_cfg: Detection configuration
    :param objects_cfg: Objects configuration
    :return: True if the robot sees the gate, False otherwise
    """
    for i in range(10):
        time.sleep(1)
        small_rotation.execute_small_rot_positive(30, 1)
        map, number_gate_pillars, goal, _ = world_analysis(rob, detection_cfg, objects_cfg, fill_map=False)

        if number_gate_pillars != 0:
            return True
    return False


def get_garage_sides(rob, map, detection_cfg, objects_cfg):
    """
    Return the fitted sides of the garage. The sides are defined by a point and a vector.
    :param rob: Robot object
    :param map: Map object
    :param detection_cfg: Detection configuration
    :param objects_cfg: Objects configuration
    :return: The fitted sides of the garage
    """
    # Get dimensions of the garage
    garage_width = map.conv_real_to_map(map.garage.get_width())
    garage_length = map.conv_real_to_map(map.garage.get_length())

    # Get the detected points of the garage
    points = map.garage.get_world_coordinates()

    # Convert real world parameters to map parameters
    xs = map.conv_real_to_map(points[0], add=True)
    ys = map.conv_real_to_map(points[1])

    # Fill the map with the detected points
    map.fill_in_garage([])

    # Fit the lines (we will be able to get all 3 sides of the garage or less)
    lines = []
    for i in range(2):
        # Fit a line to the points
        xs, ys, line, inliers = map.fit_line(xs, ys)
        if line is not None:
            lines.append(line)
    return lines, map  # TODO: delete map
