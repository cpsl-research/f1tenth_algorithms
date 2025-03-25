import math
import time

import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped

from .base import AutoControl, AutoControlException, set_member


class WiggleControl(AutoControl):
    def __init__(
        self,
        dt_state: float = 1.5,
        n_states: int = 4,
        speed: float = 1.0,
        angular_velocity: float = 1.0,
    ):
        super().__init__()
        self.state = 0
        self.start_time = time.time()
        self.dt_state = dt_state
        self.n_states = n_states
        self.speed = speed
        self.angular_velocity = angular_velocity

    def get_control_command(self) -> AckermannDriveStamped:
        # get ackermann command
        drive = AckermannDrive()

        # -- lateral
        set_member(drive, "steering_angle", self.get_steering_angle(self.state))
        set_member(drive, "steering_angle_velocity", float(self.angular_velocity))

        # -- longitudinal
        set_member(drive, "speed", float(self.speed))
        set_member(drive, "acceleration", 0.0)
        set_member(drive, "jerk", 0.0)

        # package in stamped message
        msg = AckermannDriveStamped()
        set_member(msg.header, "stamp", self.get_clock().now().to_msg())
        set_member(msg, "drive", drive)

        # update state
        if (time.time() - self.start_time) > self.dt_state:
            self.increment_state()

        return msg

    def increment_state(self):
        self.state = (self.state + 1) % self.n_states
        self.start_time = time.time()

    @staticmethod
    def get_steering_angle(state: int, turn_ang: float = math.pi / 4) -> float:
        if state in [0, 2]:
            ang = 0.0
        elif state == 1:
            ang = turn_ang  # turn left
        elif state == 3:
            ang = -turn_ang  # turn right
        else:
            raise NotImplementedError(state)
        return ang


def main(args=None):
    rclpy.init(args=args)
    node = WiggleControl()

    try:
        rclpy.spin(node)
    except AutoControlException as e:
        node.get_logger().error(e.message)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()
