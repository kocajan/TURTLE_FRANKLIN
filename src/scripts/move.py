import sys

import numpy as np
import math
import yaml
#from SingleMove import SingleMove
#from robot import GLOBAL_STOP

from rospy import Rate


class SingleMove:
    def __init__(self, rotation, distance, straight):
        self.rotation = rotation
        self.distance = distance
        self.straight = straight

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
    def is_straight(self):
        return self.straight

class Move:
    def __init__(self, robot, move_coords, detection_cfg):
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
        if index != prev_index and prev_index is not None: #tohle se trigne kdyz se zmeni indexy
            if prev_index != 0:#and index == 2: # jak se ma
                #return SingleMove(rotation, translation)
                return SingleMove(rotation, translation, True)
            else:
                return SingleMove(rotation - sequence[-1].get_rotation(), translation, False) #-sequence[-1].get_rotation() if not straight

    def revert_rotations(self, sequence):
        """
        turns all rotations to oposite by *-1. -45 deg is right, 45 deg is left
        :param sequence:
        :return: Nothing
        """
        for idx in range(2, len(sequence)):
            sequence[idx].set_rotation(-sequence[idx].get_rotation())

    def append_last_dummy_pixel(self):
        x = self.move_coords[-1][0]
        y = self.move_coords[-1][1]
        x_prev = self.move_coords[-2][0]
        y_prev = self.move_coords[-2][1]

        # this is diagonal, append straight
        if(abs(x-x_prev) == 1 and abs(y-y_prev) == 1):
            self.move_coords.append((self.move_coords[-1][0], self.move_coords[-1][1] + 1))
        # this is straight, append diagonal
        if(abs(x-x_prev) == 0 and abs(y-y_prev) == 1):
            self.move_coords.append((self.move_coords[-1][0] + 1, self.move_coords[-1][1] + 1))

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

        #self.move_coords.append((self.move_coords[-1][0]+1, self.move_coords[-1][1]+1)) # append last straight to properly end second-last move
        self.append_last_dummy_pixel()
        
        sequence.append(SingleMove(0, 0, False))  # dummy move to prevent out of bounds in array
        sequence.append(SingleMove(0, 0, False))  # dummy move to prevent out of bounds in array

        for pos in range(len(self.move_coords)):
            next_neighbours = self.generate_neighbours(self.move_coords[pos], prev_coord)
            for index, neigh in enumerate(next_neighbours):

                if (pos+1) < (len(self.move_coords)) and neigh == self.move_coords[pos+1] and neigh is not None:
                    if index == 0:     #up
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        rotation = 0
                        translation = translation + 1
                    elif index == 1:   #down
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        rotation = 0
                        translation = translation - 1
                    elif index == 2:   #diagonal up right
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        rotation = 45
                        translation = translation + math.sqrt(2)
                    elif index == 3:   #diagonal up left
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        rotation = -45
                        translation = translation + math.sqrt(2)
                    elif index == 4:   #diagonal down right
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        rotation = -135
                        translation = translation - math.sqrt(2)
                    elif index == 5:   #diagonal down left
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        rotation = 135
                        translation = translation - math.sqrt(2)
                    elif index == 6:   #90 deg left
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        rotation = -90
                        translation = 0
                    elif index == 7:   #90 deg left
                        if (tmp := self.check_trend(index, prev_index, rotation, translation, sequence)) is not None:
                            sequence.append(tmp)
                            translation = 0
                        rotation = 90
                        translation = 0
                    prev_index = index
                    break
            prev_coord = self.move_coords[pos]
            last_compensation = 0 if prev_index == 0 else sequence[-1].get_rotation()
        sequence.append(self.check_trend(index, 10, rotation - last_compensation, translation, sequence))
        sequence.pop() # remove last move
        self.revert_rotations(sequence)
        return sequence

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
                self.robot.cmd_velocity(angular=0.6)
                act_rot = self.robot.get_odometry()[2]
                if act_rot >= 0.08:
                    break

        elif direction == 1:
            while not self.robot.is_shutting_down() and not self.robot.get_stop():
                self.robot.cmd_velocity(angular=-0.6)
                act_rot = self.robot.get_odometry()[2]
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
                self.robot.cmd_velocity(angular = 0.6)
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
                self.robot.cmd_velocity(angular = -0.6)
                prev_rot = act_rot
            self.compensation(-1)

    def execute_small_rot_positive(self, degrees):
        rotation = SingleMove(-degrees, 0, None)

        if not self.robot.get_stop():
            # print('here', move.get_distance(), move.get_rotation())
            self.rotate_degrees(rotation.get_rotation())
            self.go_straight(rotation.get_distance(), np.sign(rotation.get_distance()))
        else:
            sys.exit()
    def execute_small_rot_negative(self, degrees):
        rotation = SingleMove(degrees, 0, None)

        if not self.robot.get_stop():
            # print('here', move.get_distance(), move.get_rotation())
            self.rotate_degrees(rotation.get_rotation())
            self.go_straight(rotation.get_distance(), np.sign(rotation.get_distance()))
        else:
            sys.exit()

if __name__ == '__main__':
    #path =  [(250, 0), (251, 1), (252, 2), (253, 3), (253, 4), (253, 5), (253, 6), (253, 7), (253, 8), (253, 9), (253, 10), (253, 11), (253, 12), (253, 13), (253, 14), (253, 15), (253, 16), (253, 17), (253, 18), (253, 19), (253, 20), (253, 21), (253, 22), (253, 23), (253, 24), (253, 25), (253, 26), (253, 27), (253, 28), (253, 29), (253, 30), (253, 31), (253, 32), (253, 33), (253, 34), (253, 35), (253, 36), (253, 37), (253, 38), (253, 39), (253, 40), (253, 41), (253, 42), (253, 43), (253, 44), (253, 45), (253, 46), (253, 47), (253, 48), (253, 49), (253, 50), (253, 51), (253, 52), (253, 53), (253, 54), (253, 55), (253, 56), (253, 57), (253, 58), (253, 59), (253, 60), (253, 61), (253, 62), (253, 63), (253, 64), (253, 65), (253, 66), (253, 67)]
    #path = [(250, 0), (251, 1), (252, 2), (253, 3), (254, 4), (255, 5), (256, 6), (257, 7), (258, 8), (259, 9), (260, 10), (261, 11), (262, 12), (263, 13), (264, 14), (265, 15), (266, 16), (267, 17), (268, 18), (269, 19), (270, 20), (271, 21), (271, 22), (271, 23), (271, 24), (271, 25), (271, 26), (271, 27), (271, 28), (271, 29), (271, 30), (271, 31), (271, 32), (271, 33), (271, 34), (271, 35), (271, 36), (271, 37), (271, 38), (271, 39), (271, 40), (271, 41), (271, 42), (271, 43), (271, 44), (271, 45), (271, 46), (271, 47), (271, 48), (271, 49), (271, 50), (271, 51), (271, 52), (271, 53), (271, 54), (271, 55), (271, 56), (271, 57), (271, 58), (271, 59), (271, 60), (271, 61), (271, 62), (271, 63), (271, 64), (271, 65), (271, 66), (271, 67), (271, 68), (271, 69), (271, 70), (271, 71), (271, 72), (271, 73), (271, 74), (271, 75), (271, 76), (271, 77), (271, 78), (271, 79), (271, 80), (271, 81), (271, 82), (271, 83), (271, 84), (271, 85), (271, 86), (271, 87), (271, 88), (271, 89), (271, 90), (271, 91), (271, 92), (271, 93), (271, 94), (271, 95), (271, 96), (271, 97), (271, 98), (271, 99), (271, 100), (271, 101), (271, 102), (271, 103), (271, 104), (271, 105), (271, 106), (271, 107), (271, 108), (271, 109), (271, 110), (271, 111), (271, 112), (271, 113), (271, 114), (271, 115), (271, 116), (271, 117), (271, 118), (271, 119), (271, 120), (271, 121), (271, 122), (271, 123), (271, 124), (271, 125), (271, 126), (271, 127), (271, 128), (271, 129), (271, 130), (271, 131), (271, 132), (271, 133), (271, 134), (271, 135), (271, 136), (271, 137), (271, 138), (271, 139), (271, 140), (271, 141), (271, 142), (271, 143), (271, 144), (271, 145), (271, 146), (271, 147), (271, 148), (271, 149), (271, 150), (271, 151), (271, 152), (271, 153), (271, 154), (271, 155), (271, 156), (271, 157), (271, 158), (271, 159), (271, 160), (271, 161), (271, 162), (271, 163), (271, 164), (271, 165), (271, 166), (271, 167), (271, 168), (271, 169), (271, 170), (271, 171), (271, 172), (271, 173), (271, 174), (271, 175), (271, 176), (271, 177), (271, 178), (271, 179), (271, 180), (271, 181), (271, 182), (271, 183), (271, 184), (271, 185), (271, 186), (271, 187), (271, 188), (271, 189), (271, 190), (271, 191), (271, 192), (271, 193), (271, 194), (271, 195), (271, 196), (271, 197), (271, 198), (271, 199), (271, 200), (271, 201), (271, 202), (271, 203), (271, 204), (271, 205), (271, 206), (271, 207), (271, 208), (270, 209), (269, 210), (268, 211), (267, 212), (266, 213), (265, 214), (264, 215), (263, 216), (262, 217), (261, 218), (260, 219), (259, 220), (258, 221), (257, 222), (256, 223), (255, 224), (254, 225), (253, 226), (252, 227), (251, 228), (250, 229), (249, 230), (248, 231), (247, 232), (246, 233), (245, 234), (245, 235), (245, 236), (244, 237), (245, 238), (245, 239), (245, 240), (246, 241), (247, 242), (248, 242), (249, 242), (250, 243), (251, 242), (252, 242), (253, 242), (254, 241), (255, 240), (256, 239), (257, 238), (258, 237), (259, 236), (260, 235), (261, 235), (262, 235), (263, 235), (264, 235), (265, 235), (266, 235), (267, 235), (268, 235), (269, 235), (270, 235), (271, 235)]
    #path = [(250, 0), (251, 1), (252, 2), (253, 3), (254, 4), (255, 5), (256, 6), (257, 7), (258, 8), (259, 9), (260, 10), (261, 11), (262, 12), (263, 13), (264, 14), (265, 15), (266, 16), (267, 17), (268, 18), (269, 19), (270, 20), (271, 21), (272, 22), (273, 23), (274, 24), (275, 25), (276, 26), (277, 27), (278, 28), (279, 29), (280, 30), (281, 31), (282, 32), (282, 33), (282, 34), (282, 35), (282, 36), (282, 37), (282, 38), (282, 39), (282, 40), (282, 41), (282, 42), (282, 43), (282, 44), (282, 45), (282, 46), (282, 47), (282, 48), (282, 49), (282, 50), (282, 51), (282, 52), (282, 53), (282, 54), (282, 55), (282, 56), (282, 57), (282, 58), (282, 59), (282, 60), (282, 61), (282, 62), (283, 63), (284, 64), (285, 65), (286, 66), (287, 67), (288, 68), (289, 69), (290, 70), (291, 71), (292, 72), (293, 73), (293, 74), (293, 75), (293, 76), (293, 77), (293, 78), (293, 79), (293, 80), (293, 81), (293, 82), (293, 83), (293, 84), (293, 85), (292, 86), (291, 87), (290, 88), (289, 89), (288, 90), (287, 91), (286, 92), (285, 93), (284, 94), (284, 95), (284, 96), (284, 97), (284, 98), (284, 99), (284, 100), (284, 101), (284, 102), (284, 103), (284, 104), (284, 105), (284, 106), (284, 107), (284, 108), (284, 109), (284, 110), (284, 111), (284, 112), (284, 113), (284, 114)]
    #path = [(250, 0), (251, 1), (252, 2), (253, 3), (254, 4), (255, 5), (256, 6), (257, 7), (258, 8), (259, 9), (260, 10), (261, 11), (262, 12), (263, 13), (264, 14), (265, 15), (266, 16), (267, 17), (268, 18), (269, 19), (269, 20), (269, 21), (269, 22), (269, 23), (269, 24), (269, 25), (269, 26), (269, 27), (269, 28), (269, 29), (269, 30), (269, 31), (269, 32), (269, 33), (269, 34), (269, 35), (269, 36), (269, 37), (269, 38), (270, 39), (271, 40), (272, 41), (273, 42), (274, 43), (275, 44), (276, 45), (277, 46), (278, 47), (279, 48), (279, 49), (279, 50), (279, 51), (279, 52), (279, 53), (279, 54), (279, 55), (279, 56), (279, 57), (279, 58), (279, 59), (279, 60), (279, 61), (279, 62), (279, 63), (279, 64), (279, 65), (278, 66), (277, 67), (276, 68), (275, 69), (274, 70), (273, 71), (272, 72), (271, 73), (270, 74), (269, 75), (269, 76), (269, 77), (269, 78), (269, 79), (269, 80), (269, 81), (269, 82), (269, 83), (269, 84), (269, 85), (269, 86), (269, 87), (269, 88), (269, 89), (269, 90), (269, 91), (269, 92), (269, 93), (269, 94), (269, 95), (269, 96), (269, 97), (269, 98), (269, 99), (269, 100), (269, 101), (269, 102), (269, 103), (269, 104), (269, 105), (269, 106), (269, 107), (269, 108)]
    #path = [(250, 0), (251, 1), (252, 2), (253, 3), (254, 4), (255, 5), (256, 6), (257, 7), (258, 8), (259, 9), (260, 10), (261, 11), (262, 12), (263, 13), (264, 14), (265, 15), (266, 16), (267, 17), (268, 18), (269, 19), (270, 20), (271, 21), (272, 22), (273, 23), (274, 24), (275, 25), (276, 26), (277, 27), (278, 28), (279, 29), (280, 30), (281, 31), (282, 32), (283, 33), (284, 34), (285, 35), (286, 36), (287, 37), (288, 38), (289, 39), (290, 40), (291, 41), (292, 42), (293, 43), (294, 44), (294, 45), (294, 46), (294, 47), (294, 48), (294, 49), (294, 50), (294, 51), (294, 52), (294, 53), (294, 54), (294, 55), (294, 56), (294, 57), (294, 58), (294, 59), (294, 60), (294, 61), (294, 62), (294, 63), (294, 64), (294, 65), (294, 66), (294, 67), (294, 68), (294, 69), (294, 70), (294, 71), (294, 72), (294, 73), (294, 74), (294, 75), (294, 76), (294, 77), (294, 78), (294, 79), (294, 80), (294, 81), (294, 82), (294, 83), (294, 84), (294, 85), (294, 86), (294, 87), (294, 88), (294, 89), (294, 90), (294, 91), (294, 92), (294, 93), (294, 94), (294, 95), (294, 96), (294, 97), (294, 98), (294, 99), (294, 100), (294, 101), (294, 102), (294, 103), (294, 104), (294, 105), (294, 106)]
    path = [(250, 0), (251, 1), (252, 2), (253, 3), (254, 4), (255, 5), (256, 6), (256, 7), (256, 8), (256, 9), (256, 10), (256, 11), (256, 12), (256, 13), (256, 14), (256, 15), (256, 16), (256, 17), (256, 18), (256, 19), (256, 20), (256, 21), (256, 22), (256, 23), (256, 24), (256, 25), (256, 26), (256, 27), (256, 28), (256, 29), (256, 30), (256, 31), (256, 32), (256, 33), (256, 34), (256, 35), (256, 36), (256, 37), (256, 38), (256, 39), (256, 40), (256, 41), (256, 42), (256, 43), (256, 44), (256, 45), (256, 46), (256, 47), (256, 48), (256, 49), (256, 50), (256, 51), (256, 52), (256, 53), (256, 54), (256, 55), (256, 56), (256, 57), (256, 58), (256, 59), (256, 60), (256, 61), (256, 62), (256, 63), (256, 64), (256, 65), (256, 66), (256, 67), (257, 68), (258, 69), (259, 70), (260, 71), (261, 72), (262, 73), (263, 74), (264, 75), (265, 76), (266, 77), (267, 78), (268, 79), (269, 80), (270, 81), (271, 82), (272, 83), (273, 84), (274, 85), (275, 86), (276, 87), (277, 88), (278, 89), (279, 90), (280, 91), (281, 92), (282, 93), (283, 94), (284, 95), (285, 96), (286, 97), (286, 98), (286, 99), (286, 100), (286, 101), (286, 102), (286, 103), (286, 104), (286, 105), (286, 106), (286, 107), (286, 108), (286, 109), (286, 110), (286, 111), (286, 112), (286, 113), (286, 114), (286, 115), (286, 116), (286, 117), (286, 118), (286, 119), (286, 120), (286, 121), (286, 122), (286, 123), (286, 124), (285, 125), (284, 126), (283, 127), (282, 128), (281, 129), (280, 130), (279, 131), (278, 132), (277, 133), (276, 134), (275, 135), (274, 136), (273, 137), (272, 138), (271, 139), (270, 140), (269, 141), (268, 142), (267, 143), (266, 144), (265, 145), (264, 146), (263, 147), (262, 147), (261, 147), (260, 147), (259, 147), (258, 147), (257, 147), (256, 147)]
    #path = [(250, 0), (249, 1), (248, 2), (247, 3), (246, 4), (245, 5), (244, 6), (243, 7), (242, 8), (241, 9), (240, 10), (239, 11), (238, 12), (237, 13), (236, 14), (235, 15), (234, 16), (233, 17), (232, 18), (232, 19), (232, 20), (232, 21), (232, 22), (232, 23), (232, 24), (232, 25), (232, 26), (232, 27), (232, 28), (232, 29), (232, 30), (232, 31), (232, 32), (232, 33), (232, 34), (232, 35), (232, 36), (232, 37), (232, 38), (232, 39), (232, 40), (232, 41), (232, 42), (232, 43), (232, 44), (232, 45), (232, 46), (232, 47), (232, 48), (232, 49), (232, 50), (232, 51), (232, 52), (232, 53), (232, 54), (232, 55), (232, 56), (232, 57), (232, 58), (232, 59), (232, 60), (232, 61), (232, 62), (232, 63), (232, 64), (232, 65), (232, 66), (232, 67), (232, 68), (232, 69), (232, 70), (232, 71), (232, 72), (232, 73), (232, 74), (232, 75), (232, 76), (232, 77), (232, 78), (232, 79), (232, 80), (232, 81), (232, 82), (232, 83), (232, 84), (232, 85), (232, 86), (232, 87), (232, 88), (232, 89), (232, 90), (232, 91), (231, 92), (230, 93), (229, 94), (228, 95), (227, 96), (226, 97), (225, 98), (224, 99), (223, 100), (222, 101), (221, 102), (220, 103), (219, 104), (218, 105), (217, 106), (217, 107), (217, 108), (217, 109), (217, 110), (217, 111), (217, 112), (217, 113), (217, 114), (217, 115), (217, 116), (217, 117), (217, 118), (217, 119), (217, 120), (217, 121), (217, 122), (217, 123), (217, 124), (217, 125), (217, 126), (217, 127), (217, 128), (217, 129), (217, 130), (217, 131), (217, 132), (217, 133), (218, 134), (219, 135), (220, 136), (221, 137), (222, 138), (223, 139), (224, 140), (225, 141), (226, 142), (227, 143), (228, 144), (229, 145), (230, 146), (231, 147), (232, 148)]
    detection_cfg = yaml.safe_load(open('conf/detection.yaml', 'r'))
    tmp = Move(None, path, detection_cfg)
    #tmp.move_sequence()
    tmp.execute_move()
