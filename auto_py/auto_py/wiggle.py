import rclpy
import ackermann_msgs.msg
import time

from .base import set_member, AutoControl, AutoControlException


class WiggleControl(AutoControl):
    """
    Basic control algorithm to make the f1tenth wiggle around
    """

    def __init__(self, wiggle_time: float = 1.0, angle: float = 0.524):
        super().__init__()

        self.wiggle_time = wiggle_time
        self.t_maneuver_start = None
        self.polarity = 1
        self.angle = angle

    def get_control_command(self) -> ackermann_msgs.msg.AckermannDriveStamped:
        # get ackermann command
        drive = ackermann_msgs.msg.AckermannDrive()
        # -- lateral
        ang = self.set_steering_angle()
        set_member(drive, "steering_angle", ang)
        set_member(drive, "steering_angle_velocity", 0.5)
        # -- longitudinal
        set_member(drive, "speed", 0.5)
        set_member(drive, "acceleration", 0.0)
        set_member(drive, "jerk", 0.0)

        # package in stamped message
        msg = ackermann_msgs.msg.AckermannDriveStamped()
        set_member(msg.header, "stamp", self.get_clock().now().to_msg())
        set_member(msg, "drive", drive)

        return msg

    def set_steering_angle(self) -> None:
        # increment the time in this maneuver
        if self.t_maneuver_start is None:
            self.t_maneuver_start = 1e-9 * self.get_clock().now().nanoseconds
        t_now = 1e-9 * self.get_clock().now().nanoseconds

        # determine when it's time to switch the mode
        if (t_now - self.t_maneuver_start) > self.wiggle_time:
            self.polarity *= -1
            self.t_maneuver_start = t_now

        # return the desired angle
        return self.polarity * self.angle
 

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

