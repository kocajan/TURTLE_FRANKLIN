import cv2 as cv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap

# just to get rid of annoying warnings
matplotlib.use('TkAgg')


class Visualizer:
    def __init__(self, rgb_img, point_cloud, map, processed_rgb_imgs, detection_cfg: dict):
        self.rgb_img = rgb_img
        self.processed_rgb_imgs = processed_rgb_imgs
        self.point_cloud = point_cloud
        self.map = map
        self.gate = self.map.get_gate()
        self.garage = self.map.get_garage()
        self.obstacles = self.map.get_obstacles()
        self.detection_cfg = detection_cfg

    def visualize_rgb(self):
        """
        Visualize the RGB image
        """
        copy_img = self.rgb_img.copy()

        # draw contours of the gate
        if self.gate is not None:
            gate_contours = self.gate.get_contours()
            cv.drawContours(copy_img, gate_contours, -1, (255, 0, 0), 1)

        # draw contours of the garage
        if self.garage is not None:
            garage_contours = self.garage.get_contour()
            cv.drawContours(copy_img, garage_contours, -1, (255, 0, 0), 1)

        # draw contours of the obstacles
        for obstacle in self.obstacles:
            cv.drawContours(copy_img, obstacle.get_contour(), -1, (255, 0, 0), 1)

        # draw bounding rect of the gate
        if self.gate is not None:
            rects = self.gate.get_bounding_rect()
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
        if self.gate is not None and self.gate.get_num_slopes() == 2:
            p1, p2 = self.gate.get_lowest_points()
            center = self.gate.get_center()
            angle = self.gate.get_orientation()
            cv.circle(copy_img, p1, 5, (0, 0, 255), -1)
            cv.circle(copy_img, p2, 5, (0, 0, 255), -1)
            cv.circle(copy_img, center, 5, (255, 0, 0), -1)

            # Draw a connecting line between the lowest points
            cv.line(copy_img, p1, p2, (0, 255, 0), 2)

            # Draw a perpendicular vector to the gate
            cv.line(copy_img, center, (int(center[0] + 100 * np.cos(angle + np.pi / 2)),
                                  int(center[1] + 100 * np.sin(angle + np.pi / 2))), (0, 255, 0), 2)

            # Draw a parallel vector to the x axis in the center of the gate
            cv.line(copy_img, center, (int(center[0] + 100),
                                  int(center[1])), (255, 0, 0), 2)

        for output in self.processed_rgb_imgs:
            output = cv.cvtColor(output, cv.COLOR_GRAY2BGR)
            final = np.concatenate((copy_img, output), axis=1)
            cv.imshow("RGB", final)
            cv.waitKey(0)

    def visualize_map(self, path=None):
        """
        Visualize the map
        """
        color_map = ListedColormap(["white", "black", "yellow", "magenta", "red", "green", "blue"])
        color_map = [color_map]
        world_map = self.map.get_world_map()

        if path is not None:
            path_col = self.detection_cfg["map"]["id"]["path"]
            for point in path:
                world_map[point[0], point[1]] = path_col

        n = len(color_map)
        fig, axs = plt.subplots(1, n, figsize=(10, 10), constrained_layout=True, squeeze=False)
        for [ax, cmap] in zip(axs.flat, color_map):
            psm = ax.pcolormesh(world_map, cmap=cmap, rasterized=True, vmin=0, vmax=5)
            fig.colorbar(psm, ax=ax)
        plt.show()
