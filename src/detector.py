import numpy as np
import cv2 as cv

from .objects import Obstacle, Garage, Gate
from .map import Map


class Detector:
    def __init__(self, map: Map, rgb_img: np.ndarray, point_cloud: np.ndarray, detection_cfg: dict, objects_cfg: dict):
        """
        Detector objects are used to process the RGB image and the point cloud to detect objects.
        :param map: The map object.
        :param rgb_img: The RGB image.
        :param point_cloud: The point cloud.
        :param detection_cfg: The detection configuration.
        :param objects_cfg: The objects configuration.
        """
        self.map = map
        self.rgb_img = rgb_img
        self.point_cloud = point_cloud
        self.detection_cfg = detection_cfg
        self.objects_cfg = objects_cfg
        self.processed_rgb = []
        self.processed_point_cloud = []

    # BEGIN: RGB image processing
    # CORE METHODS
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

            # Save output
            self.processed_rgb.append(mask)

            # Find contours
            contours, _ = cv.findContours(mask, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)

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

    # HELPER METHODS
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

            # Find out which dimension is the width and which is the height and make them smaller
            width_scale = self.detection_cfg['width_scale']
            height_scale = self.detection_cfg['height_scale']
            if rect[1][0] > rect[1][1]:
                rect = ((rect[0][0], rect[0][1]), (rect[1][0] * height_scale, rect[1][1] * width_scale), rect[2])
            else:
                rect = ((rect[0][0], rect[0][1]), (rect[1][0] * width_scale, rect[1][1] * height_scale), rect[2])

            bounding_rects.append(rect)
        return bounding_rects

    def set_up_garage(self, contours: list) -> None:
        """
        Create Garage object for the found garage of certain color and add it to the map.
        :param contours: The contours of the garage.
        :return: None
        """
        min_area = self.detection_cfg['min_detected_area']

        # Find a contour with the biggest area
        big_contours = []
        for cnt in contours:
            area = cv.contourArea(cnt)
            if area > min_area:
                big_contours.append(cnt)

        # Garage not found
        if len(big_contours) == 0:
            return

        # Create the garage object
        cf_garage = self.objects_cfg['garage']
        garage = Garage(cf_garage['length'], cf_garage['width'], cf_garage['height'], cf_garage['color'],
                        big_contours)

        # Add the garage to the map
        self.map.set_garage(garage)

    def set_up_gate(self, contours: list, bounding_rects: list) -> None:
        """
        Create Gate object for the found gate of certain color and add it to the map.
        :param contours: The contours (in this case simplified by the bounding box of the actual contour) of pillars
                        of the gate.
        :param bounding_rects: The bounding rectangles of the gate.
        :return: None
        """
        if len(bounding_rects) > 2:
            # Make number of contours equal to 2 (the gate has 2 pillars) by keeping the biggest ones
            bounding_rects.sort(key=lambda rect: rect[1][0] * rect[1][1], reverse=True)
            bounding_rects = bounding_rects[:2]
            print("WARNING: More than 2 pillars of the gate found. Keeping the biggest ones.")
        elif len(bounding_rects) == 0:
            return
        # We have found at least one pillar of the gate
        # Calculate the lowest points of the gate
        lowest_points = []
        if len(bounding_rects) == 2:
            lowest_points.append(self.calculate_lowest_point_of_rect(bounding_rects[0], True))
            lowest_points.append(self.calculate_lowest_point_of_rect(bounding_rects[1], False))

        # Create the gate object
        cf_gate = self.objects_cfg['gate']
        cf_garage = self.objects_cfg['garage']

        garage_dimensions_lwh = (cf_garage['length'], cf_garage['width'], cf_garage['height'])

        gate = Gate(cf_gate['pillars_width'], cf_gate['pillars_height'], cf_gate['color'], contours, bounding_rects,
                    cf_gate['pillars_distance'], lowest_points, garage_dimensions_lwh)

        # Calculate the gate's orientation
        gate.calculate_orientation_rgb()

        # Add the gate to the map
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
            self.map.set_obstacle(obstacle)

    def calculate_lowest_point_of_rect(self, rect: tuple, left: bool) -> tuple:
        """
        Calculate the lowest and the leftmost/rightmost point of the rectangle.
        :param rect: The rectangle.
        :param left: True if the left point is to be calculated, False if the right point is to be calculated.
        :return: The lowest point of the rectangle.
        """
        # Get corners of the rectangle
        corners = cv.boxPoints(rect)
        corners = np.int0(corners)

        # Set left or right point on the bottom part of the image
        img_height = self.rgb_img.shape[0]
        img_width = self.rgb_img.shape[1]

        if left:
            ref_point = (0, img_height)
        else:
            ref_point = (img_width, img_height)

        # Get the lowest point of the corners
        lowest_point = corners[0]
        min_dist = self.calculate_distance(ref_point, corners[0])
        for corner in corners:
            dist = self.calculate_distance(ref_point, corner)
            if dist < min_dist:
                min_dist = dist
                lowest_point = corner

        return lowest_point

    @staticmethod
    def calculate_distance(point1: tuple, point2: tuple) -> float:
        """
        Calculate the distance between two points.
        :param point1: The first point.
        :param point2: The second point.
        :return: The distance between the points.
        """
        return np.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2)

    # END: RGB image processing

    # BEGIN: Point cloud processing
    # CORE METHODS
    def process_point_cloud(self) -> None:
        """
        Process the point cloud to detect how far things are.
        :return: None
        """
        # Rotate point cloud
        self.rotate_point_cloud()

        # Get world coordinates of the objects in the RGB image
        # Obstacles
        for obstacle in self.map.get_obstacles():
            w_coords = self.get_world_coordinates_using_bounding_rect(obstacle.get_bounding_rect())
            if w_coords is not None:
                obstacle.set_world_coordinates(w_coords)
            else:
                self.map.get_obstacles().remove(obstacle)
        # Gate
        gate = self.map.get_gate()
        if gate is not None:
            w_coords = []
            for pillar in gate.get_bounding_rects():
                w_coord = self.get_world_coordinates_using_bounding_rect(pillar)
                if w_coord is not None:
                    w_coords.append(w_coord)
                else:
                    gate.remove_pillar(pillar)
            gate.set_world_coordinates(w_coords)

        # Garage
        garage = self.map.get_garage()
        if garage is not None:
            # Get rid of outliers
            w_coords = self.get_world_coordinates_using_contours(garage.get_contours())
            w_coords = np.array(w_coords)
            if w_coords is not None and len(w_coords) != 0:
                w_coords = np.unique(w_coords, axis=1)
                garage.set_world_coordinates(w_coords)
            else:
                self.map.set_garage(None)

    # HELPER METHODS
    def get_world_coordinates_using_contours(self, contours: list) -> tuple:
        """
        Get the world coordinates of the object using the contours.
        :param contours: The contours.
        :return: The world coordinates of the object. (xs, ys)
        """
        # Get the points in the contours
        points_in_contours = np.empty((0, 3))

        for contour in contours:
            # Connect the points from every iteration to one list
            points = self.get_pc_points_in_contours([contour])
            points_in_contours = np.vstack((points_in_contours, points))

        # Get rid of nan values
        points_in_contours = self.get_rid_of_nan(points_in_contours)

        # Get rid of outliers
        points_in_contours = self.get_rid_of_outliers(points_in_contours)

        return points_in_contours[:, 0], points_in_contours[:, 2]

    def get_rid_of_outliers(self, points: np.ndarray) -> np.ndarray:
        """
        Get rid of outliers in the given points.
        :param points: The points.
        :return: The points without outliers.
        """
        # Calculate median of the x and y coordinates
        x = np.median(points[:, 0])
        y = np.median(points[:, 2])

        # Calculate the distance of each point to the median
        distances = np.sqrt((points[:, 0] - x) ** 2 + (points[:, 2] - y) ** 2)

        # Get rid of points that are further away than the threshold
        threshold = self.detection_cfg['outlier_threshold']
        points = points[distances < threshold]

        return points

    def get_world_coordinates_using_bounding_rect(self, bounding_rect: tuple) -> tuple:
        """
        Get the world coordinates of the object using the bounding rectangle.
        :param bounding_rect: The bounding rectangle.
        :return: The world coordinates of the object.
        """
        points_in_bounding_rect = self.get_pc_points_in_bounding_rect(bounding_rect)

        # Get rid of nan values
        points_in_bounding_rect = self.get_rid_of_nan(points_in_bounding_rect)

        # If there are no points in the bounding rectangle, return None
        if len(points_in_bounding_rect) == 0:
            print("WARNING: No valid points in the point cloud in the bounding rectangle!")
            return None
        elif len(points_in_bounding_rect) < self.detection_cfg['min_valid_points_in_bounding_rect']:
            print("WARNING: Not enough valid points in the point cloud in the bounding rectangle!")
            return None

        # Calculate median of the x and y coordinates
        x = np.median(points_in_bounding_rect[:, 0])
        y = np.median(points_in_bounding_rect[:, 2])

        return x, y

    def get_pc_points_in_bounding_rect(self, bounding_rect: tuple) -> np.ndarray:
        """
        Get the points of the point cloud that are in the given bounding rectangle.
        :param bounding_rect: The bounding rectangle.
        :return: The points of the point cloud that are in the bounding rectangle.
        """

        # Get the corners of the bounding rectangle
        corners = cv.boxPoints(bounding_rect)
        corners = np.int0(corners)

        return self.get_pc_points_in_contours([corners])

    def get_pc_points_in_contours(self, contours: list) -> np.ndarray:
        """
        Get the points of the point cloud that are in the given contour.
        :param contours: The contours.
        :return: The points of the point cloud that are in the bounding rectangle.
        """
        # Create img of the same dimensions as the RGB image, filled with zeros
        img_with_contours = np.zeros((self.rgb_img.shape[0], self.rgb_img.shape[1]), np.uint8)

        # Draw the contours on the img
        cv.drawContours(img_with_contours, contours, 0, 255, -1)

        # Get the points of the point cloud that are  in the bounding rectangle
        points_in_contours = self.processed_point_cloud[np.where(img_with_contours == 255)]

        return points_in_contours

    def rotate_point_cloud(self) -> None:
        """
        Rotate the point cloud to make it easier to process.
        :return: None
        """
        # Get the rotation matrix (3D rotation respect to the x-axis)
        angle = np.deg2rad(self.detection_cfg['point_cloud_rotation'])
        rotation_matrix = np.array([[1, 0, 0],
                                    [0, np.cos(angle), -np.sin(angle)],
                                    [0, np.sin(angle), np.cos(angle)]])
        # Rotate the point cloud
        self.processed_point_cloud = np.dot(self.point_cloud, rotation_matrix)

    @staticmethod
    def get_rid_of_nan(array: np.ndarray) -> np.ndarray:
        """
        Get rid of NaN values in the array.
        :param array: The array.
        :return: The array without NaN values.
        """
        array = np.array(array)
        dims = len(array.shape)

        return array[~np.isnan(array).any(axis=dims-1)]

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

    def set_processed_rgb(self, processed_rgb):
        self.processed_rgb = processed_rgb

    def set_processed_point_cloud(self, processed_point_cloud):
        self.processed_point_cloud = processed_point_cloud

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

    def get_processed_rgb(self):
        return self.processed_rgb

    def get_processed_point_cloud(self):
        return self.processed_point_cloud
