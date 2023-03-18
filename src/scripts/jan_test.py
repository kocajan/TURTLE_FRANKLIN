import numpy as np
import cv2 as cv
import yaml

from objects import Robot, Obstacle, Gate, Garage
from detector import Detector
from map import Map
from visualizer import Visualizer
from a_star2 import astar


def image_man(img: np.ndarray) -> None:

    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))

    map = Map((5, 5), 0.01, detection_cfg)

    det = Detector(map, img, None, detection_cfg, objects_cfg)
    det.process_rgb()

    vis = Visualizer(img, None, map, det.get_processed_rgb(), detection_cfg)
    vis.visualize_rgb()


def map_visualization_test() -> None:
    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))

    # create a map
    dims = detection_cfg['map']['dimensions']
    res = detection_cfg['map']['resolution']
    map = Map(dims, res, detection_cfg)

    # create a garage
    len = objects_cfg['garage']['length']
    wid = objects_cfg['garage']['width']
    hei = objects_cfg['garage']['height']
    col = objects_cfg['garage']['color']

    garage = Garage(len, wid, hei, None, col)
    garage.set_world_coordinates((0, 3))
    garage.set_orientation(0)

    # create a gate
    wid = objects_cfg['gate']['slopes_width']
    hei = objects_cfg['gate']['slopes_height']
    slopes_dis = objects_cfg['gate']['slopes_distance']
    col = objects_cfg['gate']['color']

    gate = Gate(wid, hei, col, None, [], slopes_dis, None)
    gate.set_world_coordinates([(0, 3), (0.6, 3)])

    # create a robot
    rad = objects_cfg['robot']['radius']
    hei = objects_cfg['robot']['height']
    col = objects_cfg['robot']['color']

    robot = Robot(rad, col, 'black')
    robot.set_world_coordinates((0, 0))

    # create multiple obstacles
    rad = objects_cfg['obstacle']['radius']
    hei = objects_cfg['obstacle']['height']

    obstacle = Obstacle(rad, hei, 'red', None, None)
    obstacle.set_world_coordinates((-1, 1))

    obstacle2 = Obstacle(rad, hei, 'blue', None, None)
    obstacle2.set_world_coordinates((1, 1))

    obstacle3 = Obstacle(rad, hei, 'green', None, None)
    obstacle3.set_world_coordinates((0, 1))

    # add the objects to the map
    map.set_garage(garage)
    map.set_robot(robot)
    map.set_obstacle(obstacle)
    map.set_obstacle(obstacle2)
    map.set_obstacle(obstacle3)
    map.set_gate(gate)

    # visualize the map
    # map.visualize()

    # visualize the map in the world
    map.fill_world_map()

    cv.rectangle(map.world_map, (100, 100), (100 + 300, 100 + 50), 3, -1)

    path = map.find_way((0, 250), (280, 280))

    vis = Visualizer(None, None, map, None, detection_cfg)
    vis.visualize_map(path)


def main():
    test = "map"
    if test == "image":
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
    elif test == "map":
        map_visualization_test()


if __name__ == '__main__':
    main()

