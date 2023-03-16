import numpy as np
import cv2 as cv
import yaml

from classes import Map, Obstacle, Garage, Gate


def find_bounding_rects(img: np.ndarray, contours: list, min_area: int) -> list:
    """
    Find the smallest bounding rectangles of the contours.
    :param img: The image to draw bounding boxes on.
    :param contours: The contours to process.
    :param min_area: The minimum required area of the contour.
    :return: A list of the bounding rectangles.
    """
    bounding_rects = []
    for cnt in contours:
        if cv.contourArea(cnt) < min_area:
            continue
        rect = cv.minAreaRect(cnt)
        bounding_rects.append(rect)
        box = cv.boxPoints(rect)
        box = np.int0(box)
        cv.drawContours(img, [box], 0, (0, 0, 255), 1)
    return bounding_rects


def extract_info_from_image(map: Map, detection_cfg: dict, objects_cfg: dict) -> None:
    """
    Find objects of interest in the image, create objects for them, fill it with extracted data and add to the map.
    :param img: The image to process.
    :param map: The map object.
    :param detection_cfg: The detection config.
    :param objects_cfg: The objects config.
    :return: None
    """
    # Convert to HSV
    img = map.get_image()
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

    for color in detection_cfg['colors'].values():
        # Threshold the HSV image to get only the selected color
        lower = np.array(color['lower'])
        upper = np.array(color['upper'])
        mask = cv.inRange(hsv, lower, upper)
        output = cv.bitwise_and(img, img, mask=mask)

        # Get rid of noise
        kernel = np.ones((5, 5), np.uint8)
        output = cv.morphologyEx(output, cv.MORPH_OPEN, kernel)

        # Make the image binary
        output = cv.cvtColor(output, cv.COLOR_BGR2GRAY)

        # Find contours
        contours, _ = cv.findContours(output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

        # Draw contours
        copy_img = img.copy()
        cv.drawContours(copy_img, contours, -1, (0, 255, 0), 1)

        # Get the smallest rectangle that contains the contour (not for the garage)
        bounding_rects = None
        if color['tag'] != objects_cfg['garage']['color']:
            bounding_rects = find_bounding_rects(copy_img, contours, detection_cfg['min_detected_area'])

        # Process what has been found
        if color['tag'] in objects_cfg['obstacle']['color']:
            # Obstacles
            set_up_obstacles(bounding_rects, map, color['tag'], objects_cfg)
        elif color['tag'] == objects_cfg['gate']['color']:
            # Gate
            set_up_gate(bounding_rects, map, objects_cfg)
        elif color['tag'] == objects_cfg['garage']['color']:
            # Garage
            set_up_garage(contours, map, objects_cfg)
        else:
            # Unknown
            print('ERROR: Unknown color')
            exit(-1)

        # Visualize the result
        output = cv.cvtColor(output, cv.COLOR_GRAY2BGR)
        final = np.concatenate((copy_img, output), axis=1)
        cv.imshow(color['tag'], final)
        cv.waitKey(0)


def set_up_garage(contours: list, map: Map, objects_cfg: dict) -> None:
    """
    Create Garage object for the found garage of certain color and add it to the map.
    :param contours: The contours of the garage.
    :param map: The map object.
    :param objects_cfg: The 'objects config'.
    :return: None
    """
    # Find a contour with the biggest area
    biggest_contour = None
    biggest_area = 0
    for cnt in contours:
        area = cv.contourArea(cnt)
        if area > biggest_area and area > 2000:
            biggest_area = area
            biggest_contour = cnt

    # Garage not found
    if biggest_contour is None:
        return

    # Create the garage object
    cf_garage = objects_cfg['garage']
    garage = Garage(cf_garage['length'], cf_garage['width'], cf_garage['height'], cf_garage['color'], biggest_contour)

    # Add the garage to the map
    map.set_garage(garage)


def set_up_gate(contours: list, map: Map, objects_cfg: dict) -> None:
    """
    Create Gate object for the found gate of certain color and add it to the map.
    :param contours: The contours (in this case simplified by the bounding box of the actual contour) of slopes
                    of the gate.
    :param map: The map object.
    :param objects_cfg: The 'objects config'.
    :return: None
    """
    if len(contours) > 2:
        # Make number of contours equal to 2 (the gate has 2 slopes) by removing what?
        # TODO: Think it through
        pass
    elif len(contours) == 0:
        return
    # We have found at least one slope of the gate
    cf_gate = objects_cfg['gate']
    gate = Gate(cf_gate['slopes_width'], cf_gate['slopes_height'], cf_gate['color'], contours, cf_gate['slopes_distance'])
    map.set_gate(gate)


def set_up_obstacles(contours: list, map: Map, color: str, objects_cfg: dict) -> None:
    """
    Create Obstacle objects for the found obstacles of certain color and add them to the map.
    :param contours: The contours (in this case simplified by the bounding box of the actual contour) of the obstacles.
    :param map: The map object.
    :param objects_cfg: The 'objects config'.
    :return: None
    """
    for contour in contours:
        # Create the obstacle object
        cf_obstacle = objects_cfg['obstacle']
        obstacle = Obstacle(cf_obstacle['radius'], cf_obstacle['height'], color, contour)
        # Add the obstacle to the map
        map.add_obstacle(obstacle)


def calculate_image_to_world_transformations(map: Map) -> None:
    """
    Calculate the transformations from image to world coordinates and vice versa.
    :return: None
    """
    # Calculate real world coordinates (relative to the robot):
    #   - obstacles:
    calculate_obstacles_world_coordinates(map)
    #   - gate:
    calculate_gate_orientation(map)


def calculate_gate_orientation(map: Map) -> None:
    """
    Calculate the orientation of the gate.
    :param map: The map object.
    :return: None
    """
    gate = map.get_gate()
    img = map.get_image()

    if gate is None or gate.get_num_slopes() != 2:
        return

    # Get the lowest points of the slopes
    x1, y1 = calculate_lowest_point_of_rect(gate.contours[0])
    x2, y2 = calculate_lowest_point_of_rect(gate.contours[1])

    # Calculate center of the gate
    center = (int((x1 + x2) / 2), int((y1 + y2) / 2))

    # Calculate the angle between the gate and the horizontal axis
    angle = np.arctan2(y1 - y2, x1 - x2)

    # Draw the lowest points and the center
    cv.circle(img, (x1, y1), 5, (0, 0, 255), -1)
    cv.circle(img, (x2, y2), 5, (0, 0, 255), -1)
    cv.circle(img, center, 5, (255, 0, 0), -1)

    # Draw a connecting line between the lowest points
    cv.line(img, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # Draw a perpendicular vector to the gate
    cv.line(img, center, (int(center[0] + 100 * np.cos(angle + np.pi / 2)),
                          int(center[1] + 100 * np.sin(angle + np.pi / 2))), (0, 255, 0), 2)

    # Draw a parallel vector to the x axis in the center of the gate
    cv.line(img, center, (int(center[0] + 100),
                          int(center[1])), (255, 0, 0), 2)

    # Show the result
    cv.imshow('Gate', img)

    print("ANGLE (DEG): ", angle * 180 / np.pi)
    map.get_gate().set_orientation(angle)


def calculate_lowest_point_of_rect(rect: tuple) -> tuple:
    """
    Calculate the lowest point of the rectangle.
    :param rect: The rectangle.
    :return: The lowest point of the rectangle.
    """
    # Get corners of the rectangle
    corners = cv.boxPoints(rect)
    corners = np.int0(corners)

    # Get the lowest point of the corners
    lowest_point = corners[0]
    for point in corners:
        if point[1] > lowest_point[1]:
            lowest_point = point

    return lowest_point


def calculate_obstacles_world_coordinates(map: Map) -> None:
    """
    Calculate the real world coordinates of the obstacles.
    :param map: The map object.
    :return: None
    """
    obstacles = map.get_obstacles()

    for obstacle in obstacles:
        pass


def process_image(img: np.ndarray) -> Map:
    """
    Process the image and show the result.
    :param img: The image to process.
    :return: None
    """
    # Load configs
    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))

    # Create the map object
    map = Map(img, detection_cfg['map']['dimensions'], detection_cfg['map']['resolution'])

    # Find objects of interest
    extract_info_from_image(map, detection_cfg, objects_cfg)

    # Calculate dependencies between image objects and real world objects
    calculate_image_to_world_transformations(map)


if __name__ == '__main__':
    for i in range(16):
        if i == 3:
            continue
        img = cv.imread(f'camera/shoot1/{i}.png')
        process_image(img)
        cv.waitKey(0)
        cv.destroyAllWindows()

    for i in range(9):
        img = cv.imread(f'camera/shoot2/{i}.png')
        process_image(img)
        cv.waitKey(0)
        cv.destroyAllWindows()

