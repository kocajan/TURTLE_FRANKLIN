import numpy as np
import cv2 as cv
from .turtlebot import Turtlebot
from rospy import Rate

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

    def take_rgb_img(self):
        """
        Take RGB photo and return it as numpy array.
        :return: RGB photo as numpy array.
        """
        # Wait for the camera to setup
        self.wait_for_rgb_image()
        
        # Capture RGB image
        rgb = self.get_rgb_image()

        return rgb

    def take_point_cloud(self):
        """
        Get point cloud from the robot.
        :return: Point cloud as numpy array.
        """
        # Wait for the camera to setup
        self.wait_for_point_cloud()

        # Capture point cloud
        pc = self.get_point_cloud()
        return pc

    def take_depth_img(self):
        """
        Get depth image from the robot.
        :return: Depth image as numpy array.
        """
        # Wait for the camera to setup
        self.wait_for_depth_image()
        # Capture depth image
        depth = self.get_depth_image()
        return depth

    def stop_motors(self):
        """
        Force stop the robot.
        """
        self.cmd_velocity(linear=0)
        self.cmd_velocity(angular=0)

    def bumper_cb(self, msg):
        """
        Bumper callback.
        :param msg: Bumper message.
        """
        self.stop = True
        self.stop_motors()

    def timer_cb(self):
        if self.is_there_anything_close():
            self.set_stop(True)
            self.stop_motors()

    def is_there_anything_close(self):
        """
        Check if there is anything close to the robot.
        :return: True if there is something close, False otherwise.
        """
        x_range = (-0.3, 0.3)
        z_range = (0.1, 3.0)

        # Get the point cloud
        pc = self.take_point_cloud()

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
        # Show unique values up to 40
        unique = np.unique(image[np.where(image < 31)])
        print("----------------------------------")
        print("Unique values: \n", unique)
        print("Number of unique values: ", len(unique))
        print("----------------------------------")
        image1 = image[np.where(image <= 18)]
        image2 = image1[np.where(image1 > 0)]

        num_danger_points = len(image2)

        if num_danger_points > 350:
            print(num_danger_points, ' DANGER POINTS!')
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


if __name__ == '__main__':
    my_robot = Robot(1, 1, 1)
    path = [(250, 0), (251, 1), (252, 2), (253, 3), (253, 4), (253, 5), (253, 6), (253, 7), (253, 8), (253, 9), (253, 10), (253, 11), (253, 12), (253, 13), (253, 14), (253, 15), (253, 16), (253, 17), (253, 18), (253, 19), (253, 20), (253, 21), (253, 22), (253, 23), (253, 24), (253, 25), (253, 26), (253, 27), (253, 28), (253, 29), (253, 30), (253, 31), (253, 32), (253, 33), (253, 34), (253, 35), (253, 36), (253, 37), (253, 38), (253, 39), (253, 40), (253, 41), (253, 42), (253, 43), (253, 44), (253, 45), (253, 46), (253, 47), (253, 48), (253, 49), (253, 50), (253, 51), (253, 52), (253, 53), (253, 54), (253, 55), (253, 56), (253, 57), (253, 58), (253, 59), (253, 60), (253, 61), (253, 62), (253, 63), (253, 64), (253, 65), (253, 66), (253, 67)]
    tmp = move(my_robot, path)
    # tmp.move_sequence()
    tmp.execute_move()

