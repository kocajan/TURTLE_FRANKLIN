import numpy as np
import cv2 as cv
import yaml

from objects import Robot, Obstacle, Gate, Garage
from detector import Detector
from map import Map
from visualizer import Visualizer


def image_man(img: np.ndarray) -> None:

    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))

    map = Map((5, 5), 0.01, detection_cfg)

    det = Detector(map, img, None, detection_cfg, objects_cfg)
    det.process_rgb()

    vis = Visualizer(img, None, map, det.get_processed_rgb())
    vis.visualize_rgb()


def main():
    for i in range(16):
        if i == 3:
            continue
        img = cv.imread(f'camera/shoot1/{i}.png')
        image_man(img)
        cv.waitKey(0)
        cv.destroyAllWindows()

    for i in range(9):
        img = cv.imread(f'camera/shoot2/{i}.png')
        image_man(img)
        cv.waitKey(0)
        cv.destroyAllWindows()


if __name__ == '__main__':
    main()

