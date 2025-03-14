import sys
import math
import numpy as np

from rospy import Rate
from .robot import Robot

class SingleMove:
    def __init__(self, rotation: float, distance: float, straight: bool):
        """
        Single move objects are used to store information about a single move.
        :param rotation: Rotation angle in radians.
        :param distance: Distance in meters.
        :param straight: True if the robot should go straight, False if it should rotate.
        """
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


class RegulatedMove:
    def __init__(self, robot: Robot, move_cfg: dict):
        """
        RegulatedMove object is used to move the robot along a path.
        :param robot: Robot object.
        :param move_cfg: Configuration file.
        """
        self.robot = robot
        self.move_cfg = move_cfg
        self.rate = Rate(10)

    def odometry_hard_reset(self) -> None:
        """
        Hard reset the robot's odometry.
        :return: None
        """
        self.robot.reset_odometry()
        self.robot.wait_for_odometry()
        self.robot.reset_odometry()

    def go(self, path: list) -> None:
        """
        Go along the path.
        The algorithm is based on automatic control. The robot will go straight at constant speed so the only regulated
        movement is the rotation. When the robot moves, we can calculate its velocity vector. The error we want to
        minimize is the angle between the velocity vector and the vector from the robot to the current set-point.
        Only every n-th path point is used as a set-point for the robot's regulator. The set-point
        is updated when the robot is close enough to the current set-point. The robot will stop when it reaches the
        last point in the path. PI regulator is used for the rotation.
        Parameters mentioned above can be changed in the configuration file.
        :param path: Path to follow.
        :return: None
        """
        # Load the parameters from the configuration file
        # -> Regulator parameters
        P = self.move_cfg['regulator']['P']
        I = self.move_cfg['regulator']['I']
        linear_velocity = self.move_cfg['regulator']['linear_velocity']
        anti_windup_constant = self.move_cfg['regulator']['anti_windup']

        # -> Lowpass filter parameters
        lowpass_const = self.move_cfg['lowpass']['const']
        lowpass_const_increment = self.move_cfg['lowpass']['const_increment']

        # -> Setpoint parameters and others
        setpoint_idx_step = self.move_cfg['setpoint']['idx_step']
        setpoint_distance_threshold = self.move_cfg['setpoint']['distance_threshold']
        goal_distance_threshold = self.move_cfg['goal_distance_threshold']
        x_offset = self.move_cfg['x_offset']

        # Initialize the necessary variables
        sum = 0
        setpoint_idx = setpoint_idx_step

        # Offset the path
        path = self.subtract_offset(path, x_offset)

        # Set the first set-point (IMPORTANT)
        setpoint = (-path[setpoint_idx][0], path[setpoint_idx][1])

        # Preset the odometry values
        prev_odometry_values = [(0, 0)]

        # Hard reset the odometry
        self.odometry_hard_reset()

        while not self.robot.is_shutting_down() and not self.robot.get_stop():
            # Get the robot's odometry in cm [(y, x, angle) -> (x, y)]
            odometry_cm = self.robot.get_odometry()[:2]*100
            odometry_cm = odometry_cm[::-1]

            # We wnat to keep only the last n odometry values
            n = 100
            if len(prev_odometry_values) > n:
                prev_odometry_values.pop(0)

            # Calculate the error
            error = self.calculate_error(setpoint, odometry_cm, prev_odometry_values)

            # Add the current odometry value to the list of previous odometry values
            prev_odometry_values.append(odometry_cm)

            # Anti-windup
            if abs(sum) > anti_windup_constant:
                sum = 0
            else:
                sum = error + sum

            # Update the set-point if the robot is close enough to the current set-point
            setpoint_distance = self.calculate_distance(odometry_cm, setpoint)
            if setpoint_distance < setpoint_distance_threshold:
                # If we are close enough to the last point of the path (goal), we end the movement
                if path[setpoint_idx] == path[-1] and setpoint_distance < goal_distance_threshold:
                    break
                if setpoint_idx != -1:
                    setpoint_idx += setpoint_idx_step
                if setpoint_idx >= len(path):
                    setpoint_idx = -1
                setpoint = (-path[setpoint_idx][0], path[setpoint_idx][1])

            # Execute the movement
            self.robot.cmd_velocity(linear=linear_velocity, angular=lowpass_const*P*error + I*sum) # speed 0.05
            self.rate.sleep()

            # Increment the lowpass constant if it is less than 1
            if lowpass_const < 1:
                lowpass_const += lowpass_const_increment
        

    def calculate_error(self, setpoint: list, current_odometry_value: np.ndarray,
                        previous_odometry_values: np.ndarray) -> float:
        """
        Calculate the error for the robot's regulator.
        It is represented as the difference between velocity vector and the vector from the robot to the goal.
        The odometry value is in the form of (y, x).
        :param setpoint: Setpoint for the robot's regulator.
        :param current_odometry_value: Current odometry value.
        :param previous_odometry_values: List of previous odometry values.
        :return: Error for the robot's regulator. (the angle)
        """
        # Calculate the difference between the current odometry value and the previous odometry value
        # Take the n-th previous odometry value (it will represent the velocity vector)
        velocity_vector = self.calculate_difference(current_odometry_value, previous_odometry_values, n=1)

        # Calculate the vector from the robot to the goal
        vector_to_goal = np.array(setpoint) - np.array(current_odometry_value)

        # Calculate the angle (in radians) between the vector from the robot to the goal and the robot's velocity vector
        at1 = math.atan2(vector_to_goal[1], vector_to_goal[0])
        at2 = math.atan2(velocity_vector[1], velocity_vector[0])

        # Make sure that the angle is between 0 and 2*pi (the atan2 function returns values between -pi and pi)
        if at1 < 0:
            at1 = 2*np.pi+at1
        if at2 < 0:
            at2 = 2*np.pi+at2

        # Calculate the angle between the two vectors
        angle = at2-at1

        return angle

    def rotate_degrees_no_compensation(self, degrees: float, speed: float) -> None:
        """
        Rotate the robot for a given number of degrees without compensating for the robot's rotation.
        :param degrees: Number of degrees to rotate the robot.
        :param speed: Speed of the robot's rotation. (from 0 to 1)
        :return: None
        """
        damping = 0
        self.odometry_hard_reset()
        if abs(degrees) > 40:
            offset = 0.08
        else:
            offset = 0

        if degrees > 0:
            prev_rot = 0
            goal = np.radians(degrees)

            while not self.robot.is_shutting_down() and not self.robot.get_stop():
                act_rot = self.robot.get_odometry()[2]
                if act_rot >= goal-offset or abs(act_rot - prev_rot) > 1:
                    print(act_rot)
                    break
                self.robot.cmd_velocity(angular = np.exp(damping)*speed)
                if damping >= -0.45:
                    damping -= 0.01
                prev_rot = act_rot

        elif degrees < 0:
            prev_rot = 0
            goal = np.radians(degrees)

            while not self.robot.is_shutting_down() and not self.robot.get_stop():
                act_rot = self.robot.get_odometry()[2]
                if act_rot <= goal+offset or abs(act_rot - prev_rot) > 1:
                    break
                self.robot.cmd_velocity(angular = -np.exp(damping)*speed)
                if damping >= -0.45:
                    damping -= 0.01
                prev_rot = act_rot

    def execute_small_rot_positive(self, degrees: float, speed: float) -> None:
        """
        Rotate the robot for a given number of degrees. (in the positive direction)
        :param degrees: Number of degrees to rotate the robot.
        :param speed: Speed of the robot's rotation. (from 0 to 1)
        :return: None
        """
        rotation = SingleMove(-degrees, 0, None)

        if not self.robot.get_stop():
            self.rotate_degrees_no_compensation(rotation.get_rotation(), speed)
        else:
            sys.exit()

    def execute_small_rot_negative(self, degrees: float, speed: float) -> None:
        """
        Rotate the robot for a given number of degrees. (in the negative direction)
        :param degrees: Number of degrees to rotate the robot.
        :param speed: Speed of the robot's rotation. (from 0 to 1)
        :return: None
        """
        rotation = SingleMove(degrees, 0, None)

        if not self.robot.get_stop():
            self.rotate_degrees_no_compensation(rotation.get_rotation(), speed)
        else:
            sys.exit()

    @staticmethod
    def calculate_difference(current_odometry_value: np.ndarray, previous_odometry_values: np.ndarray, n=1) -> tuple:
        """
        Calculate the difference between the current odometry value and the previous odometry value.
        Take the n-th previous odometry value.
        The odometry value is in the form of (y, x).
        :param current_odometry_value: Current odometry value.
        :param previous_odometry_values: List of previous odometry values.
        :param n: Take the n-th previous odometry value.
        :return: Difference between the current odometry value and the previous odometry value.
        """
        return current_odometry_value - previous_odometry_values[-n]

    @staticmethod
    def calculate_distance(pt1: list, pt2: list) -> float:
        """
        Calculate the distance between two points.
        :param pt1: First point.
        :param pt2: Second point.
        :return: Distance between two points.
        """
        return np.linalg.norm(np.array(pt2) - np.array(pt1))

    @staticmethod
    def subtract_offset(path: list, x_offset: int) -> list:
        """
        Subtract the x offset from the path.
        :param path: Path to subtract the offset from.
        :param x_offset: Offset to subtract.
        :return: Path with the offset subtracted.
        """
        new_path = list()
        for item in path:
            new_path.append((item[0]-x_offset, item[1]))
        return new_path