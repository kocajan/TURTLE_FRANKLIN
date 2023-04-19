import sys

import numpy as np
import math
import yaml
#from SingleMove import SingleMove
#from robot import GLOBAL_STOP

from rospy import Rate


class SingleMove:
    # TODO: delete this? -------------------------------------------------------
    move_coords = list(list())  #2D array of moving coords
    # TODO: --------------------------------------------------------------------

    def __init__(self, rotation, distance):
        self.rotation = rotation
        self.distance = distance

    # SETTERS
    def set_rotation(self, rotation):
        self.rotation = rotation

    def set_distance(self, distance):
        self.distance = distance

    # GETTERS
    def get_rotation(self):
        return self.rotation

    def get_distance(self):
        return self.distance


class Move:
    # TODO: delete this? -------------------------------------------------------
    move_coords = list(list())  #2D array of moving coords
    # TODO: --------------------------------------------------------------------

    def __init__(self, robot, move_coords, detection_cfg):
        self.detection_cfg = detection_cfg                  # TODO: delete this? (not used)
        self.move_coords = move_coords
        self.robot = robot
        self.rate = Rate(10)

    def execute_move(self):
        """
        Executes the move sequence.
        """
        moves_to_execute = self.move_sequence()
        print('number of moves: ', len(moves_to_execute))
        for mv in moves_to_execute:
            print('moves: ', mv.get_distance(), mv.get_rotation())
        for index, move in enumerate(moves_to_execute):
            if not self.robot.get_stop():
                #print('here', move.get_distance(), move.get_rotation())
                self.rotate_degrees(move.get_rotation())
                self.go_straight(move.get_distance(), np.sign(move.get_distance()))
            else:
                sys.exit()

    #format of neighbours is UP, DOWN, DIAGONAL_UP_RIGHT, DIAGONAL_UP_LEFT, DIAGONAL_DOWN_RIGHT, DIAGONAL_DOWN_LEFT, RIGHT_ANGLE_LEFT, RIGHT_ANGLE_RIGHT
    #zaroven se nemuzu divat na predchozi pozici, asi to chce do fce jako argument
    def generate_neighbours(self, current_pos, prev_pos):
        """
        Generates neighbours of the current position.
        :param current_pos: Current position.
        :param prev_pos: Previous position.
        :return: List of neighbours.
        """
        neighbours = list()
        neighbours.append((current_pos[0], current_pos[1] + 1))                 # up
        if prev_pos is not None:
            neighbours.append((current_pos[0], current_pos[1] - 1))             # down
        elif prev_pos is None:
            neighbours.append(None)

        neighbours.append((current_pos[0] + 1, current_pos[1] + 1))             # diagonal up right
        neighbours.append((current_pos[0] - 1, current_pos[1] + 1))             # diagonal up left

        if prev_pos is not None:
            neighbours.append((current_pos[0] + 1, current_pos[1] - 1))         # diagonal down right
            neighbours.append((current_pos[0] - 1, current_pos[1] - 1))         # diagonal down left
        elif prev_pos is None:
            neighbours.append(None)
            neighbours.append(None)

        neighbours.append((current_pos[0] - 1, current_pos[1]))                 # left
        neighbours.append((current_pos[0] + 1, current_pos[1]))                 # right

        for index, neighbour in enumerate(neighbours):
            if neighbour == prev_pos and prev_pos is not None :
                #neighbours.remove(neighbour) # removing previous position, it is invalid neighbour
                neighbour = (-100, -100)   # mark as invalid instead of deletion
        return neighbours

    def check_trend(self, index, prev_index, rotation, translation, sequence):
        """
        Checks the trend of the move sequence.
        :param index: Index of the current neighbour.
        :param prev_index: Index of the previous neighbour.
        :param rotation: Current rotation.
        :param translation: Current translation.
        :param sequence: Move sequence.
        :return: Returns the compensation move.
        """
        if index != prev_index and prev_index is not None:
            if rotation != 0:
                tmp = SingleMove(-rotation, translation) # mozna se to posere
            elif rotation == 0:
                tmp = SingleMove(sequence[-1].get_rotation(), translation)
            return tmp
        return None

    def get_compensation_rotation(self, sequence):
        """
        Returns the compensation rotation.
        :param sequence: Move sequence.
        :return: Returns the compensation rotation.
        """
        if (abs(sequence[-1].get_rotation()) - abs(sequence[-2].get_rotation())) == 0:
            return 0
        else:
            return -sequence[-1].get_rotation()

    #creates SingleMove classes. SingleMove classes are then executed
    def move_sequence(self):
        """
        Creates the move sequence.
        :return: Returns the move sequence.
        """
        rotation = 0
        translation = 0
        sequence = list()
        prev_coord = None
        prev_index = None
        prev_rotation = 0   # TODO: delete this? (not used) as well as trend?

        #TODO vyresit posledni node, ta se jenom apenduje, bude stejna jako vsechny ostatni
        sequence.append(SingleMove(0, 0))  # dummy move to prevent out of bounds in array
        sequence.append(SingleMove(0, 0))  # dummy move to prevent out of bounds in array

        for pos in range(len(self.move_coords)):
            next_neighbours = self.generate_neighbours(self.move_coords[pos], prev_coord)
            for index, neigh in enumerate(next_neighbours):
                
                if (pos+1) < (len(self.move_coords)) and neigh == self.move_coords[pos+1] and neigh is not None:
                    #print('pos, idx ', pos, index)
                    if index == 0:     #up
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            #print('here1', tmp.get_distance())
                            translation = 0
                        rotation = 0 + self.get_compensation_rotation(sequence)#- abs(sequence[-1].get_rotation())#- prev_rotation
                        translation = translation + 1
                    elif index == 1:   #down
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        rotation = 0 + self.get_compensation_rotation(sequence)#- abs(sequence[-1].get_rotation())
                        translation = translation - 1
                    elif index == 2:   #diagonal up right
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = 45 + self.get_compensation_rotation(sequence)#- abs(sequence[-1].get_rotation())
                        translation = translation + 1
                        #break
                    elif index == 3:   #diagonal up left
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = -45 + self.get_compensation_rotation(sequence)#- abs(sequence[-1].get_rotation())
                        translation = translation + 1
                        #break
                    elif index == 4:   #diagonal down right
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = -135 + self.get_compensation_rotation(sequence)#- abs(sequence[-1].get_rotation())
                        translation = translation - 1
                        #break
                    elif index == 5:   #diagonal down left
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = 135 + self.get_compensation_rotation(sequence)#- abs(sequence[-1].get_rotation())
                        translation = translation - 1
                        #break
                    elif index == 6:   #90 deg left
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = -90 + self.get_compensation_rotation(sequence)#- abs(sequence[-1].get_rotation())
                        translation = 0
                        #break
                    elif index == 7:   #90 deg left
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        trend = index
                        rotation = 90 + self.get_compensation_rotation(sequence)#- abs(sequence[-1].get_rotation())
                        translation = 0
                        #break
                    prev_index = index
                    prev_rotation = abs(rotation)
                    break
            prev_coord = self.move_coords[pos]
        sequence.append(self.check_trend(index, 10, -rotation, translation, sequence)) # pridane minus pico
        #print('here ', sequence[1].get_distance())
        return sequence
    # length in centimeters

    def go_straight(self, length, dir):
        slow_start_cnt = 0
        slowdown = False
        odometry_x_y = []
        #path_integrated = list()
        self.odometry_hard_reset()
        while not self.robot.is_shutting_down() and not self.robot.get_stop():
            #print(slow_start_cnt)
            if slow_start_cnt < 1.0 and not slowdown:
                slow_start_cnt += 0.15
            elif slowdown and slow_start_cnt >= 0.25: # 0.17
                slow_start_cnt -= 0.1
                print('FVSDVSVS')

            odometry_x_y = self.robot.get_odometry()[:1]
            if abs(odometry_x_y[0]) >= (length/100):
                break
            self.robot.cmd_velocity(linear=(0.2*dir)*slow_start_cnt)                # TODO add as yaml const
            self.rate.sleep()
            #print((abs(odometry_x_y[0])/2), ((length/100)/2))
            if (abs(odometry_x_y[0])) > ((length/100)/1.8): # /1.3
                slowdown = True
        #path_integrated.append(odometry_x_y[0])
        #path_integrated.append(odometry_x_y[1])
        return None #list(odometry_x_y[0], odometry_x_y[1])

    def compensate_straight_integration_drift(self):
        additional_rotation = 0
        #pricitej zataceci error podle integrace k jiz planovanemu zatoceni
        return additional_rotation

    def odometry_hard_reset(self):
        """
        Hard reset the robot's odometry.
        """
        self.robot.reset_odometry()
        self.robot.wait_for_odometry()
        self.robot.reset_odometry()

    # this is compensation for error drift after rotation
    def compensation(self, direction):
        """
        Calculate and apply velocity compensation.
        :param direction: Direction of the robot's velocity
        """
        # TODO move compensation constant to yaml

        self.odometry_hard_reset()
        if direction == -1:
            while not self.robot.is_shutting_down() and not self.robot.get_stop():
                self.robot.cmd_velocity(angular=0.4)
                act_rot = self.robot.get_odometry()[2]
                print(act_rot)
                if act_rot >= 0.08:
                    break

        elif direction == 1:
            while not self.robot.is_shutting_down() and not self.robot.get_stop():
                self.robot.cmd_velocity(angular=-0.4)
                act_rot = self.robot.get_odometry()[2]
                print(act_rot)
                if act_rot <= -0.08:
                    break
                
    # degree input range have to be +-180 deg
    def rotate_degrees(self, degrees):
        self.odometry_hard_reset()

        if degrees > 0:
            prev_rot = 0
            goal = np.radians(degrees)

            while not self.robot.is_shutting_down() and not self.robot.get_stop():
                act_rot = self.robot.get_odometry()[2]
                #print(act_rot, prev_rot)
                if act_rot >= goal or abs(act_rot-prev_rot) > 1:
                    break
                self.robot.cmd_velocity(angular = 0.4)
                prev_rot = act_rot
            self.compensation(1)
  
        elif degrees < 0:
            prev_rot = 0
            goal = np.radians(degrees)

            while not self.robot.is_shutting_down() and not self.robot.get_stop():
                act_rot = self.robot.get_odometry()[2]
                #print(act_rot, prev_rot)
                if act_rot <= goal or abs(act_rot - prev_rot) > 1:
                    break
                self.robot.cmd_velocity(angular = -0.4)
                prev_rot = act_rot
            self.compensation(-1)


if __name__ == '__main__':
    #path =  [(250, 0), (251, 1), (252, 2), (253, 3), (253, 4), (253, 5), (253, 6), (253, 7), (253, 8), (253, 9), (253, 10), (253, 11), (253, 12), (253, 13), (253, 14), (253, 15), (253, 16), (253, 17), (253, 18), (253, 19), (253, 20), (253, 21), (253, 22), (253, 23), (253, 24), (253, 25), (253, 26), (253, 27), (253, 28), (253, 29), (253, 30), (253, 31), (253, 32), (253, 33), (253, 34), (253, 35), (253, 36), (253, 37), (253, 38), (253, 39), (253, 40), (253, 41), (253, 42), (253, 43), (253, 44), (253, 45), (253, 46), (253, 47), (253, 48), (253, 49), (253, 50), (253, 51), (253, 52), (253, 53), (253, 54), (253, 55), (253, 56), (253, 57), (253, 58), (253, 59), (253, 60), (253, 61), (253, 62), (253, 63), (253, 64), (253, 65), (253, 66), (253, 67)]
    #path = [(250, 0), (251, 1), (252, 2), (253, 3), (254, 4), (255, 5), (256, 6), (257, 7), (258, 8), (259, 9), (260, 10), (261, 11), (262, 12), (263, 13), (264, 14), (265, 15), (266, 16), (267, 17), (268, 18), (269, 19), (270, 20), (271, 21), (271, 22), (271, 23), (271, 24), (271, 25), (271, 26), (271, 27), (271, 28), (271, 29), (271, 30), (271, 31), (271, 32), (271, 33), (271, 34), (271, 35), (271, 36), (271, 37), (271, 38), (271, 39), (271, 40), (271, 41), (271, 42), (271, 43), (271, 44), (271, 45), (271, 46), (271, 47), (271, 48), (271, 49), (271, 50), (271, 51), (271, 52), (271, 53), (271, 54), (271, 55), (271, 56), (271, 57), (271, 58), (271, 59), (271, 60), (271, 61), (271, 62), (271, 63), (271, 64), (271, 65), (271, 66), (271, 67), (271, 68), (271, 69), (271, 70), (271, 71), (271, 72), (271, 73), (271, 74), (271, 75), (271, 76), (271, 77), (271, 78), (271, 79), (271, 80), (271, 81), (271, 82), (271, 83), (271, 84), (271, 85), (271, 86), (271, 87), (271, 88), (271, 89), (271, 90), (271, 91), (271, 92), (271, 93), (271, 94), (271, 95), (271, 96), (271, 97), (271, 98), (271, 99), (271, 100), (271, 101), (271, 102), (271, 103), (271, 104), (271, 105), (271, 106), (271, 107), (271, 108), (271, 109), (271, 110), (271, 111), (271, 112), (271, 113), (271, 114), (271, 115), (271, 116), (271, 117), (271, 118), (271, 119), (271, 120), (271, 121), (271, 122), (271, 123), (271, 124), (271, 125), (271, 126), (271, 127), (271, 128), (271, 129), (271, 130), (271, 131), (271, 132), (271, 133), (271, 134), (271, 135), (271, 136), (271, 137), (271, 138), (271, 139), (271, 140), (271, 141), (271, 142), (271, 143), (271, 144), (271, 145), (271, 146), (271, 147), (271, 148), (271, 149), (271, 150), (271, 151), (271, 152), (271, 153), (271, 154), (271, 155), (271, 156), (271, 157), (271, 158), (271, 159), (271, 160), (271, 161), (271, 162), (271, 163), (271, 164), (271, 165), (271, 166), (271, 167), (271, 168), (271, 169), (271, 170), (271, 171), (271, 172), (271, 173), (271, 174), (271, 175), (271, 176), (271, 177), (271, 178), (271, 179), (271, 180), (271, 181), (271, 182), (271, 183), (271, 184), (271, 185), (271, 186), (271, 187), (271, 188), (271, 189), (271, 190), (271, 191), (271, 192), (271, 193), (271, 194), (271, 195), (271, 196), (271, 197), (271, 198), (271, 199), (271, 200), (271, 201), (271, 202), (271, 203), (271, 204), (271, 205), (271, 206), (271, 207), (271, 208), (270, 209), (269, 210), (268, 211), (267, 212), (266, 213), (265, 214), (264, 215), (263, 216), (262, 217), (261, 218), (260, 219), (259, 220), (258, 221), (257, 222), (256, 223), (255, 224), (254, 225), (253, 226), (252, 227), (251, 228), (250, 229), (249, 230), (248, 231), (247, 232), (246, 233), (245, 234), (245, 235), (245, 236), (244, 237), (245, 238), (245, 239), (245, 240), (246, 241), (247, 242), (248, 242), (249, 242), (250, 243), (251, 242), (252, 242), (253, 242), (254, 241), (255, 240), (256, 239), (257, 238), (258, 237), (259, 236), (260, 235), (261, 235), (262, 235), (263, 235), (264, 235), (265, 235), (266, 235), (267, 235), (268, 235), (269, 235), (270, 235), (271, 235)]
    path = [(250, 0), (251, 1), (252, 2), (253, 3), (254, 4), (255, 5), (256, 6), (257, 7), (258, 8), (259, 9), (260, 10), (261, 11), (262, 12), (263, 13), (264, 14), (265, 15), (266, 16), (267, 17), (268, 18), (269, 19), (270, 20), (271, 21), (272, 22), (273, 23), (274, 24), (275, 25), (276, 26), (277, 27), (278, 28), (279, 29), (280, 30), (281, 31), (282, 32), (282, 33), (282, 34), (282, 35), (282, 36), (282, 37), (282, 38), (282, 39), (282, 40), (282, 41), (282, 42), (282, 43), (282, 44), (282, 45), (282, 46), (282, 47), (282, 48), (282, 49), (282, 50), (282, 51), (282, 52), (282, 53), (282, 54), (282, 55), (282, 56), (282, 57), (282, 58), (282, 59), (282, 60), (282, 61), (282, 62), (283, 63), (284, 64), (285, 65), (286, 66), (287, 67), (288, 68), (289, 69), (290, 70), (291, 71), (292, 72), (293, 73), (293, 74), (293, 75), (293, 76), (293, 77), (293, 78), (293, 79), (293, 80), (293, 81), (293, 82), (293, 83), (293, 84), (293, 85), (292, 86), (291, 87), (290, 88), (289, 89), (288, 90), (287, 91), (286, 92), (285, 93), (284, 94), (284, 95), (284, 96), (284, 97), (284, 98), (284, 99), (284, 100), (284, 101), (284, 102), (284, 103), (284, 104), (284, 105), (284, 106), (284, 107), (284, 108), (284, 109), (284, 110), (284, 111), (284, 112), (284, 113), (284, 114)]

    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    tmp = Move(None, path, detection_cfg)
    #tmp.move_sequence()
    tmp.execute_move()
