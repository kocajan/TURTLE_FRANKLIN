import numpy as np
import cv2 as cv
import yaml
# import move

from objects import Obstacle, Gate, Garage
from robot import Robot
from detector import Detector
from map import Map
from visualizer import Visualizer


def huge_test() -> None:
    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    objects_cfg = yaml.safe_load(open('conf/objects.yaml', 'r'))

    dims = detection_cfg['map']['dimensions']
    res = detection_cfg['map']['resolution']

    map = Map(dims, res, detection_cfg)

    rad = objects_cfg['robot']['radius']
    hei = objects_cfg['robot']['height']
    col = objects_cfg['robot']['color']

    rob = Robot(rad, col, 'black')
    print('robot object created')
    print('bumper initialized')

    rob.set_world_coordinates((0, 0))
    map.set_robot(rob)

    img = rob.take_rgb_img()
    pc = rob.take_point_cloud()

    det = Detector(map, img, pc, detection_cfg, objects_cfg)
    det.process_rgb()
    det.process_point_cloud()

    map.fill_world_map()

    vis = Visualizer(img, pc, map, det.get_processed_rgb(), det.get_processed_point_cloud(), detection_cfg)

    path = map.find_way((250, 0), tuple(map.get_goal()), "A_star")


    vis.visualize_rgb()
    # vis.visualize_point_cloud()
    vis.visualize_map(path=path)

    tmp = move.Move(rob, path, detection_cfg)
    print(path)
    # tmp.execute_move()


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

    rob = Robot(rad, col, 'black')
    print('robot object created')
    print('bumper initialized')

    rob.set_world_coordinates((0, 0))
    map.set_robot(rob)
    # --------------------------------------------

    det = Detector(map, img, pc, detection_cfg, objects_cfg)
    det.process_rgb()
    det.process_point_cloud()

    map.fill_world_map()
    search_algorithm = detection_cfg['map']['search_algorithm']
    path = map.find_way((250, 0), tuple(map.get_goal()), search_algorithm)

    vis = Visualizer(img, pc, map, det.get_processed_rgb(), det.get_processed_point_cloud(), detection_cfg)

    vis.visualize_rgb()
    # vis.visualize_point_cloud()
    vis.visualize_map(path=path)
    #path = [(250, 0), (251, 1), (252, 2), (253, 3), (254, 4), (255, 5), (256, 6), (257, 7), (258, 8), (259, 9), (260, 10), (261, 11), (262, 12), (263, 13), (264, 14), (265, 15), (266, 16), (267, 17), (268, 18), (269, 19), (270, 20), (271, 21), (272, 22), (273, 23), (274, 24), (275, 25), (276, 26), (277, 27), (278, 28), (279, 29), (280, 30), (281, 31), (282, 32), (282, 33), (282, 34), (282, 35), (282, 36), (282, 37), (282, 38), (282, 39), (282, 40), (282, 41), (282, 42), (282, 43), (282, 44), (282, 45), (282, 46), (282, 47), (282, 48), (282, 49), (282, 50), (282, 51), (282, 52), (282, 53), (282, 54), (282, 55), (282, 56), (282, 57), (282, 58), (282, 59), (282, 60), (282, 61), (282, 62), (283, 63), (284, 64), (285, 65), (286, 66), (287, 67), (288, 68), (289, 69), (290, 70), (291, 71), (292, 72), (293, 73), (293, 74), (293, 75), (293, 76), (293, 77), (293, 78), (293, 79), (293, 80), (293, 81), (293, 82), (293, 83), (293, 84), (293, 85), (292, 86), (291, 87), (290, 88), (289, 89), (288, 90), (287, 91), (286, 92), (285, 93), (284, 94), (284, 95), (284, 96), (284, 97), (284, 98), (284, 99), (284, 100), (284, 101), (284, 102), (284, 103), (284, 104), (284, 105), (284, 106), (284, 107), (284, 108), (284, 109), (284, 110), (284, 111), (284, 112), (284, 113), (284, 114)]

    # tmp = move.move(rob, path)
    # print(path)
    # tmp.execute_move()


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
    test = "big"
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
    elif test == "big":
        all = False
        if all:
            for i in range(15):
                img = cv.imread(f'camera/shoot3/RGB{i}.png')
                pc = np.load(f'camera/shoot3/PC{i}.npy')
                big_test(img, pc)
        else:
            img = cv.imread(f'camera/shoot3/RGB13.png')
            pc = np.load(f'camera/shoot3/PC13.npy')
            big_test(img, pc)
    elif test == "huge":
        huge_test()


if __name__ == '__main__':
    main()

