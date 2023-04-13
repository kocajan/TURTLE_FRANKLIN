import numpy as np
import cv2 as cv
import yaml
import move

from objects import Robot, Obstacle, Gate, Garage
from robot import robot
from detector import Detector
from map import Map
from visualizer import Visualizer


def big_test(img: np.ndarray, pc: np.ndarray) -> None:
    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))

    dims = detection_cfg['map']['dimensions']
    res = detection_cfg['map']['resolution']

    map = Map(dims, res, detection_cfg)

    # Set up robot -------------------------------
    rad = objects_cfg['robot']['radius']
    hei = objects_cfg['robot']['height']
    col = objects_cfg['robot']['color']

    rob = robot(rad, col, 'black')
    print('robot object created')
    print('bumper initialized')

    rob.set_world_coordinates((0, 0))
    map.set_robot(rob)
    # --------------------------------------------

    det = Detector(map, img, pc, detection_cfg, objects_cfg)
    det.process_rgb()
    det.process_point_cloud()

    map.fill_world_map()
    path = map.find_way((250, 0), tuple(map.get_goal()))

    vis = Visualizer(img, pc, map, det.get_processed_rgb(), det.get_processed_point_cloud(), detection_cfg)

    #vis.visualize_rgb()
    # vis.visualize_point_cloud()
    #vis.visualize_map(path=path)

    tmp = move.move(rob, path)
    #print(path)
    tmp.execute_move()


def image_man(img: np.ndarray, pc: np.ndarray) -> None:

    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))

    dims = detection_cfg['map']['dimensions']
    res = detection_cfg['map']['resolution']

    map = Map(dims, res, detection_cfg)

    det = Detector(map, img, pc, detection_cfg, objects_cfg)
    det.process_rgb()
    # det.process_point_cloud()

    vis = Visualizer(img, pc, map, det.get_processed_rgb(), det.get_processed_point_cloud(), detection_cfg)
    vis.visualize_rgb()
    # vis.visualize_point_cloud()


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
    wid = objects_cfg['gate']['pillars_width']
    hei = objects_cfg['gate']['pillars_height']
    pillars_dis = objects_cfg['gate']['pillars_distance']
    col = objects_cfg['gate']['color']

    gate = Gate(wid, hei, col, None, [1, 2], pillars_dis, None, (garage.get_length(), garage.get_width(), garage.get_height()))
    length = garage.get_length()
    p1 = (0, 3)
    angle = 120
    angle = np.deg2rad(angle)
    p2 = (p1[0] + np.cos(angle) * length, p1[1] + np.sin(angle) * length)
    gate.set_world_coordinates([p1, p2])
    gate.calculate_orientation()

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

    # cv.rectangle(map.world_map, (0, 50), (400, 50 + 50), 4, -1)
    # cv.rectangle(map.world_map, (100, 200), (500, 200+50), 4, -1)

    path = map.find_way((0, 250), (280, 280))

    vis = Visualizer(None, None, map, None, detection_cfg)
    vis.visualize_map(path)

    return path


def main():
    test = "image+pc+map"
    if test == "image":
        for i in range(16):
            if i == 3:
                continue
            img = cv.imread(f'camera/shoot1/{i}.png')
            image_man(img, None)
            cv.waitKey(0)
            cv.destroyAllWindows()

        for i in range(9):
            img = cv.imread(f'camera/shoot1/{i}.png')
            image_man(img, None)
            cv.waitKey(0)
            cv.destroyAllWindows()
    elif test == "map":
        map_visualization_test()
    elif test == "pc":
        for i in range(14):
            img = cv.imread(f'camera/shoot3/RGB{i}.png')
            pc = np.load(f'camera/shoot3/PC{i}.npy')
            image_man(img, pc)
    elif test == "pc1":
        img = cv.imread(f'camera/shoot3/RGB13.png')
        pc = np.load(f'camera/shoot3/PC13.npy')
        image_man(img, pc)
    elif test == "image+pc+map":
        all = False
        if all:
            for i in range(13):
                img = cv.imread(f'camera/shoot3/RGB{i}.png')
                pc = np.load(f'camera/shoot3/PC{i}.npy')
                big_test(img, pc)
        else:
            img = cv.imread(f'camera/shoot4/RGB0.png')
            pc = np.load(f'camera/shoot4/PC0.npy')
            big_test(img, pc)


if __name__ == '__main__':
    main()

