import math
import time
from typing import List

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from auto_py.perception.object_detection import ObjectDetectionClusterer
from auto_py.perception.wall_detection import WallDetectionClusterer
from sensor_msgs.msg import LaserScan

from .base import AutoControl, AutoControlException, set_member


class RoombaControl(AutoControl):
    def __init__(
        self,
        straight_speed: float = 0.5,
        reverse_speed: float = -0.25,
        reverse_time: float = 1.0,
        reverse_angle: float = 15 * math.pi / 180,
    ):
        """Initialize a controller in Roomba mode"""
        super().__init__()
        self.straight_speed = straight_speed
        self.reverse_speed = reverse_speed
        self.reverse_time = reverse_time
        self.reverse_angle = reverse_angle
        self.is_path_blocked = False
        self.reversing = False
        self._reverse_start_time = -np.inf

        self.obj_detector = ObjectDetectionClusterer()
        self.wall_detector = WallDetectionClusterer()

    def get_control_command(self) -> AckermannDriveStamped:
        # get ackermann command
        drive = AckermannDrive()
        set_member(drive, "steering_angle_velocity", 0.0)
        set_member(drive, "acceleration", 0.0)
        set_member(drive, "jerk", 0.0)

        # set reverse mode if path is blocked
        if self.is_path_blocked:
            if not self.reversing:
                self._reverse_start_time = time.time()
            self.reversing = True

        # sets angle and speed based on lidar scan
        if self.reversing:
            # go backwards and turn at a certain degree
            set_member(drive, "steering_angle", self.reverse_angle)  # 15-degree turn
            set_member(drive, "speed", self.reverse_speed)  # go backwards
        else:
            # keep going straight
            set_member(drive, "steering_angle", 0.0)
            set_member(drive, "speed", self.straight_speed)

        # package in stamped message
        msg = AckermannDriveStamped()
        set_member(msg.header, "stamp", self.get_clock().now().to_msg())
        set_member(msg, "drive", drive)

        # stop reversing after a certain time
        if self.reversing:
            if time.time() >= self._reverse_start_time + self.reverse_time:
                self.reversing = False

        return msg

    def receive_lidar(self, msg: LaserScan):
        # TODO convert to numpy array of points
        points = np.zeros((0, 2))  # 2d array of points

        # run detection algorithms
        objs = self.detect_obstacles(points)
        walls = self.detect_obstacles(points)

        # process the objs/walls
        # TODO

        # set ego state based on the results
        self.is_path_blocked = False  # TODO

    def detect_obstacles(self, points: np.ndarray) -> List[np.ndarray]:
        return self.obj_detector(points)

    def detect_walls(self, points: np.ndarray) -> List[np.ndarray]:
        return self.wall_detector(points)


# def receive_lidar_OLD(self, msg: LaserScan):
#     # --variable definition--
#     a_min = msg.angle_min
#     a_max = msg.angle_max
#     a_incr = msg.angle_increment
#     distance_list = msg.ranges
#     list_length = len(distance_list)

#     # --filtered array from coordinates--
#     self.is_path_blocked = False
#     for i in range(list_length):
#         point_angle = a_min + (i * a_incr)
#         point_angle = np.rad2deg(point_angle)
#         point_distance = distance_list[i]
#         point_coords = np.array([point_angle, point_distance])
#         self.is_path_blocked = self.is_path_blocked or is_path_blocked(point_coords)

# def is_path_blocked(point_coords):
#     angle = point_coords[0]
#     distance = point_coords[1]
#     if angle < 20:
#         if angle > -20:
#             if distance < 2:
#                 return True
#     return False


def main(args=None):
    rclpy.init(args=args)
    node = RoombaControl()

    try:
        rclpy.spin(node)
    except AutoControlException as e:
        node.get_logger().error(e.message)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
