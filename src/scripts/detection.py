import numpy as np
import cv2 as cv
import yaml

from classes import Map, Obstacle, Garage, Gate


def get_parking_spot_orientation(img: np.ndarray, piles: np.ndarray, detection_cfg: dict) -> None:
    """
    Find the orientation of the parking spot.
    :param img: The image to process.
    :param piles: Contours of the magenta piles of the parking spot
    :param detection_cfg: The detection config.
    :return: None
    """
    # find a bottom point of the piles
    bottom_point = None
    for pile_contour in piles:
        extBot = tuple(pile_contour[pile_contour[:, :, 1].argmax()][0])
        cv.circle(img, extBot, 8, (255, 255, 0), -1)
        # TODO


def find_bounding_rects(img: np.ndarray, contours: list, min_area: int) -> list:
    """
    Find the bounding rectangles of the contours.
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


def segmentation(img: np.ndarray, map: Map, detection_cfg: dict, objects_cfg: dict) -> None:
    """
    Find objects of interest in the image. Distinguish between objects of interest in the foreground and background.
    """
    # Convert to HSV
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
        if color['tag'] != objects_cfg['garage']['color']:
            bounding_rects = find_bounding_rects(copy_img, contours, detection_cfg['min_detected_area'])

        # Process what has been found
        if color['tag'] in objects_cfg['obstacle']['color']:
            # Obstacles
            set_up_obstacles(bounding_rects, map, color['tag'], objects_cfg)
        elif color['tag'] == objects_cfg['gate']['color']:
            # Gate
            set_up_gate(bounding_rects, map, color['tag'], objects_cfg)
        elif color['tag'] == objects_cfg['garage']['color']:
            # Garage
            print('Garage')
        else:
            # Unknown
            print('ERROR: Unknown color')
            exit(-1)

        # Visualize the result
        output = cv.cvtColor(output, cv.COLOR_GRAY2BGR)
        final = np.concatenate((copy_img, output), axis=1)
        cv.imshow(color['tag'], final)
        cv.waitKey(0)


def set_up_gate(contours: list, map: Map, color: str, objects_cfg: dict) -> None:
    """
    Create Gate object for the found gate of certain color and add it to the map.
    :param contours: The contours (in this case simplified by the bounding box of the actual contour) of slopes
                    of the gate.
    :param map: The map object.
    :param objects_cfg: The 'objects config'.
    :return: None
    """
    for contour in contours:
        # Create the gate object
        cf_gate = objects_cfg['gate']
        gate = Gate(cf_gate['width'], cf_gate['height'], color, contour)
        # Add the gate to the map
        map.add_gate(gate)


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
    map = Map(detection_cfg['map']['dimensions'])

    # Find objects of interest
    segmentation(img, map, detection_cfg, objects_cfg)


if __name__ == '__main__':
    for i in range(16):
        if i == 3:
            continue
        img = cv.imread(f'camera/shoot1/{i}.png')
        process_image(img)
        cv.waitKey(0)
        cv.destroyAllWindows()
