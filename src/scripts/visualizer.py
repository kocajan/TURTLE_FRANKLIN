import cv2 as cv
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.colors import ListedColormap

# just to get rid of annoying warnings
matplotlib.use('TkAgg')


class Visualizer:
    def __init__(self, rgb_img, point_cloud, map, processed_rgb_imgs):
        self.rgb_img = rgb_img
        self.processed_rgb_imgs = processed_rgb_imgs
        self.point_cloud = point_cloud
        self.map = map
        self.gate = self.map.get_gate()
        self.garage = self.map.get_garage()
        self.obstacles = self.map.get_obstacles()

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

        for output in self.processed_rgb_imgs:
            output = cv.cvtColor(output, cv.COLOR_GRAY2BGR)
            final = np.concatenate((copy_img, output), axis=1)
            cv.imshow("RGB", final)
            cv.waitKey(0)

    def visualize_map(self):
        """
        Visualize the map
        """
        cmap = ListedColormap(["white", "black", "yellow", "magenta", "red", "green", "blue"])
        world_map = self.map.get_world_map()
        self.plot_examples([cmap], world_map)

    def plot_examples(self, colormaps, data):
        """
        Helper function to plot data with associated colormap.
        """
        n = len(colormaps)
        fig, axs = plt.subplots(1, n, figsize=(10, 10), constrained_layout=True, squeeze=False)
        for [ax, cmap] in zip(axs.flat, colormaps):
            psm = ax.pcolormesh(data, cmap=cmap, rasterized=True, vmin=0, vmax=5)
            fig.colorbar(psm, ax=ax)
        plt.show()