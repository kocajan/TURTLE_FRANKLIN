import numpy as np
import cv2 as cv

from classes import Map, Obstacle, Garage, Gate


class Detector:
    def __init__(self, map: Map, rgb_img: np.ndarray, point_cloud: np.ndarray, detection_cfg: dict, objects_cfg: dict):
        self.map = map
        self.rgb_img = rgb_img
        self.point_cloud = point_cloud
        self.detection_cfg = detection_cfg
        self.objects_cfg = objects_cfg
        self.processed_rgb = []

    # BEGIN: RGB image processing
    # CORE FUNCTION
    def process_rgb(self) -> None:
        """
        Process the RGB image to detect the garage, gate, obstacles.
        :return: None
        """
        # Convert to HSV
        img = self.rgb_img
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        for color in self.detection_cfg['colors'].values():
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

            # Save output
            self.processed_rgb.append(output)

            # Find contours
            contours, _ = cv.findContours(output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

            # Get the smallest rectangle that contains the contour (not for the garage)
            bounding_rects = None
            if color['tag'] != self.objects_cfg['garage']['color']:
                bounding_rects = self.find_bounding_rects(contours)

            # Process what has been found
            if color['tag'] in self.objects_cfg['obstacle']['color']:
                # Obstacles
                self.set_up_obstacles(contours, bounding_rects, color['tag'])
            elif color['tag'] == self.objects_cfg['gate']['color']:
                # Gate
                self.set_up_gate(contours, bounding_rects)
            elif color['tag'] == self.objects_cfg['garage']['color']:
                # Garage
                self.set_up_garage(contours)
            else:
                # Unknown
                print('ERROR: Unknown color')
                exit(-1)

    # HELPER FUNCTIONS
    def find_bounding_rects(self, contours: list) -> list:
        """
        Find the smallest bounding rectangles of the contours.
        :param contours: The contours to process.
        :return: A list of the bounding rectangles.
        """
        min_area = self.detection_cfg['min_detected_area']
        bounding_rects = []
        for cnt in contours:
            if cv.contourArea(cnt) < min_area:
                continue
            rect = cv.minAreaRect(cnt)
            bounding_rects.append(rect)
        return bounding_rects

    def set_up_garage(self, contours: list) -> None:
        """
        Create Garage object for the found garage of certain color and add it to the map.
        :param contours: The contours of the garage.
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
        cf_garage = self.objects_cfg['garage']
        garage = Garage(cf_garage['length'], cf_garage['width'], cf_garage['height'], cf_garage['color'],
                        biggest_contour)

        # Add the garage to the map
        self.map.set_garage(garage)

    def set_up_gate(self, contours: list, bounding_rects: list) -> None:
        """
        Create Gate object for the found gate of certain color and add it to the map.
        :param contours: The contours (in this case simplified by the bounding box of the actual contour) of slopes
                        of the gate.
        :param bounding_rects: The bounding rectangles of the gate.
        :return: None
        """
        if len(bounding_rects) > 2:
            # Make number of contours equal to 2 (the gate has 2 slopes) by removing what?
            # TODO: Think it through
            pass
        elif len(bounding_rects) == 0:
            return
        # We have found at least one slope of the gate
        cf_gate = self.objects_cfg['gate']
        gate = Gate(cf_gate['slopes_width'], cf_gate['slopes_height'], cf_gate['color'], contours, bounding_rects,
                    cf_gate['slopes_distance'])
        self.map.set_gate(gate)

    def set_up_obstacles(self, contours: list, bounding_rects: list, color: str) -> None:
        """
        Create Obstacle objects for the found obstacles of certain color and add them to the map.
        :param contours: The contours (in this case simplified by the bounding box of the actual contour) of the obstacles.
        :param bounding_rects: The bounding rectangles of the obstacles.
        :param color: The color of the obstacles.
        :return: None
        """
        for rect in bounding_rects:
            # Create the obstacle object
            cf_obstacle = self.objects_cfg['obstacle']
            obstacle = Obstacle(cf_obstacle['radius'], cf_obstacle['height'], color, contours, rect)
            # Add the obstacle to the map
            self.map.add_obstacle(obstacle)

    # END: RGB image processing

    # BEGIN: Point cloud processing
    # CORE FUNCTION
    def process_point_cloud(self) -> None:
        """
        Process the point cloud to detect how far things are.
        :return: None
        """
        pass

    # HELPER FUNCTIONS

    # END: Point cloud processing

    # SETTERS
    def set_rgb_img(self, rgb_img):
        self.rgb_img = rgb_img

    def set_point_cloud(self, point_cloud):
        self.point_cloud = point_cloud

    def set_map(self, map):
        self.map = map

    def set_detection_cfg(self, detection_cfg):
        self.detection_cfg = detection_cfg

    def set_objects_cfg(self, objects_cfg):
        self.objects_cfg = objects_cfg

    # GETTERS
    def get_rgb_img(self):
        return self.rgb_img

    def get_point_cloud(self):
        return self.point_cloud

    def get_map(self):
        return self.map

    def get_detection_cfg(self):
        return self.detection_cfg

    def get_objects_cfg(self):
        return self.objects_cfg
