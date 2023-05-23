import numpy as np

from rospy import Rate
from .turtlebot import Turtlebot


class Robot(Turtlebot):
    def __init__(self, radius, height, color, rgb=True, depth=True, pc=True):
        super().__init__(rgb=rgb, depth=depth, pc=pc)
        self.radius = radius
        self.height = height
        self.color = color
        self.world_coordinates = None
        self.stop = False
        self.register_bumper_event_cb(self.bumper_cb)

        #super().timer_start(self.timer_cb)

    def take_rgb_img(self) -> np.ndarray:
        """
        Take RGB photo and return it as numpy array.
        :return: RGB photo as numpy array.
        """
        # Wait for the camera to setup
        self.wait_for_rgb_image()
        
        # Capture RGB image
        rgb = self.get_rgb_image()

        return rgb

    def take_point_cloud(self) -> np.ndarray:
        """
        Get point cloud from the robot.
        :return: Point cloud as numpy array.
        """
        # Wait for the camera to setup
        self.wait_for_point_cloud()

        # Capture point cloud
        pc = self.get_point_cloud()
        return pc

    def take_depth_img(self) -> np.ndarray:
        """
        Get depth image from the robot.
        :return: Depth image as numpy array.
        """
        # Wait for the camera to setup
        self.wait_for_depth_image()
        # Capture depth image
        depth = self.get_depth_image()
        return depth

    def stop_motors(self) -> None:
        """
        Force stop the robot.
        :return: None
        """
        self.cmd_velocity(linear=0)
        self.cmd_velocity(angular=0)

    def bumper_cb(self, msg) -> None:
        """
        Bumper callback.
        :param msg: Bumper message.
        :return: None
        """
        self.stop = True
        self.stop_motors()

    def timer_cb(self, event) -> None:
        """
        Timer callback.
        :param event: Timer event.
        :return: None
        """
        if self.is_there_anything_close():
            self.set_stop(True)
            self.stop_motors()
        pass

    def is_there_anything_close(self) -> bool:
        """
        Check if there is anything close to the robot.
        :return: True if there is something close, False otherwise.
        """
        import time
        print('here')
        start = time.time()
        x_range = (-0.3, 0.3)
        z_range = (0.1, 3.0)

        # Get the point cloud
        pc = self.take_point_cloud()
        print(time.time() - start)

        if pc is None:
            return False

        # Get rid of nans and infs
        pc[np.isnan(pc)] = 0
        pc[np.isinf(pc)] = 0

        # Mask out floor points
        mask = pc[:, :, 1] > x_range[0]

        # Mask point too far and close
        mask = np.logical_and(mask, pc[:, :, 2] > z_range[0])
        mask = np.logical_and(mask, pc[:, :, 2] < z_range[1])

        if np.count_nonzero(mask) <= 0:
            return False

        # Empty image
        image = np.zeros(mask.shape)

        # Assign depth i.e. distance to image
        image[mask] = np.int16(pc[:, :, 2][mask] / 3.0 * 255)
        image = image[:240, :]

        # Get rid of points that are far enough or not defined
        image1 = image[np.where(image <= 18)]
        image2 = image1[np.where(image1 > 0)]

        # Check how many points are too close
        num_danger_points = len(image2)
        if num_danger_points > 350:
            return True
        else:
            return False

    # SETTERS
    def set_radius(self, radius):
        self.radius = radius

    def set_height(self, height):
        self.height = height

    def set_color(self, color):
        self.color = color

    def set_world_coordinates(self, world_coordinates):
        self.world_coordinates = world_coordinates

    def set_stop(self, stop):
        self.stop = stop

    # GETTERS
    def get_radius(self):
        return self.radius

    def get_height(self):
        return self.height

    def get_color(self):
        return self.color

    def get_world_coordinates(self):
        return self.world_coordinates

    def get_stop(self):
        return self.stop
