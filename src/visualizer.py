import cv2 as cv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# just to get rid of annoying warnings
matplotlib.use('TkAgg')


class Visualizer:
    def __init__(self, rgb_img: np.ndarray, point_cloud: np.ndarray, map: object, processed_rgb_imgs: np.ndarray,
                 processed_pc: np.ndarray, detection_cfg: dict):
        """
        Visualizer objects are used to visualize the RGB image, point cloud and map.
        :param rgb_img: RGB image.
        :param point_cloud: Point cloud.
        :param map: Map object.
        :param processed_rgb_imgs: Processed RGB images.
        :param processed_pc: Processed point cloud.
        :param detection_cfg: Detection configuration.
        """
        self.rgb_img = rgb_img
        self.processed_rgb_imgs = processed_rgb_imgs
        self.point_cloud = point_cloud
        self.processed_point_cloud = processed_pc
        self.map = map
        self.gate = self.map.get_gate()
        self.garage = self.map.get_garage()
        self.obstacles = self.map.get_obstacles()
        self.detection_cfg = detection_cfg

    def visualize_rgb(self) -> None:
        """
        Visualize the RGB image
        :return: None
        """
        # Copy the RGB image to keep the original image
        copy_img = self.rgb_img.copy()

        # draw contours of the gate
        if self.gate is not None:
            gate_contours = self.gate.get_contours()
            cv.drawContours(copy_img, gate_contours, -1, (255, 0, 0), 1)

        # draw contours of the garage
        if self.garage is not None:
            garage_contours = self.garage.get_contours()
            for contour in garage_contours:
                cv.drawContours(copy_img, [contour], -1, (0, 0, 255), 1)

        # draw contours of the obstacles
        for obstacle in self.obstacles:
            cv.drawContours(copy_img, obstacle.get_contour(), -1, (255, 0, 0), 1)

        # draw bounding rect of the gate
        if self.gate is not None:
            rects = self.gate.get_bounding_rects()
            for rect in rects:
                box = cv.boxPoints(rect)
                box = np.int0(box)
                cv.drawContours(copy_img, [box], 0, (250, 128, 114), 2)

        # draw bounding rects of the obstacles
        for obstacle in self.obstacles:
            rect = obstacle.get_bounding_rect()
            box = cv.boxPoints(rect)
            box = np.int0(box)
            cv.drawContours(copy_img, [box], 0, (0, 255, 0), 1)

        # draw orientation of the gate
        if self.gate is not None and self.gate.get_num_pillars() == 2:
            p1, p2 = self.gate.get_lowest_points()
            center = self.gate.get_center()
            angle = self.gate.get_orientation_rgb()
            cv.circle(copy_img, p1, 5, (0, 0, 255), -1)
            cv.circle(copy_img, p2, 5, (0, 0, 255), -1)
            cv.circle(copy_img, center, 5, (255, 0, 0), -1)

            # Draw a connecting line between the lowest points
            cv.line(copy_img, p1, p2, (0, 255, 0), 2)

            # Draw a perpendicular vector to the gate
            cv.line(copy_img, center, (int(center[0] + 100 * np.cos(angle + np.pi / 2)),
                                  int(center[1] + 100 * np.sin(angle + np.pi / 2))), (0, 255, 0), 2)

            # Draw a parallel vector to the x-axis in the center of the gate
            cv.line(copy_img, center, (int(center[0] + 100),
                                  int(center[1])), (255, 0, 0), 2)

        for output in self.processed_rgb_imgs:
            output = cv.cvtColor(output, cv.COLOR_GRAY2BGR)
            final = np.concatenate((copy_img, output), axis=1)
            cv.imshow("RGB", final)
            cv.waitKey(0)

    def visualize_map(self, path=None) -> None:
        """
        Visualize the map
        :param path: path to be visualized (in the form of a list of points)
        :return: None
        """
        color_map = ListedColormap(["white", "black", "green", "pink", "yellow", "magenta", "red", "blue", "grey",
                                    "silver"])
        color_map = [color_map]
        world_map = self.map.get_world_map()

        if path is not None:
            path_col = self.detection_cfg["map"]["id"]["path"]
            for point in path:
                world_map[point[1], point[0]] = path_col

        n = len(color_map)
        fig, axs = plt.subplots(1, n, figsize=(10, 10), constrained_layout=True, squeeze=False)
        for [ax, cmap] in zip(axs.flat, color_map):
            psm = ax.pcolormesh(world_map, cmap=cmap, rasterized=True, vmin=0, vmax=self.detection_cfg["map"]["max_id"])
            fig.colorbar(psm, ax=ax)
        plt.show()

    def visualize_point_cloud(self) -> None:
        """
        Visualize the point cloud
        :return: None
        """
        # Get the x, y and z coordinates of the point cloud
        xs = self.point_cloud[:, :, 0]
        ys = self.point_cloud[:, :, 1]
        zs = self.point_cloud[:, :, 2]

        # Get the x, y and z coordinates of the processed point cloud
        xs_p = self.processed_point_cloud[:, :, 0]
        ys_p = self.processed_point_cloud[:, :, 1]
        zs_p = self.processed_point_cloud[:, :, 2]

        # Take only every nth point
        n = 5
        xs = xs[::n, ::n]
        ys = ys[::n, ::n]
        zs = zs[::n, ::n]

        xs_p = xs_p[::n, ::n]
        ys_p = ys_p[::n, ::n]
        zs_p = zs_p[::n, ::n]

        # Swap coords to get the correct orientation
        xs, zs = zs, xs
        zs, ys = -ys, zs

        xs_p, zs_p = zs_p, xs_p
        zs_p, ys_p = -ys_p, zs_p

        # Draw a 3D graph of the point cloud
        fig = plt.figure()

        # Draw both point clouds
        ax = fig.add_subplot(111, projection='3d')
        ax.scatter(xs, ys, zs, c='r', s=0.1)
        ax.scatter(xs_p, ys_p, zs_p, c='b', s = 0.1)
        ax.set_xlabel('Y')
        ax.set_ylabel('X')
        ax.set_zlabel('Z')

        # find the min and max of the point clouds
        min_x = min(np.nanmin(xs), np.nanmin(xs_p))
        max_x = max(np.nanmax(xs), np.nanmax(xs_p))
        min_y = min(np.nanmin(ys), np.nanmin(ys_p))
        max_y = max(np.nanmax(ys), np.nanmax(ys_p))
        min_z = min(np.nanmin(zs), np.nanmin(zs_p))
        max_z = max(np.nanmax(zs), np.nanmax(zs_p))

        # set the limits of the graph
        ax.set_xlim(min_x, max_x)
        ax.set_ylim(min_y, max_y)
        ax.set_zlim(min_z, max_z)
        plt.show()
