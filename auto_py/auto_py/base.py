import importlib
import typing

import std_msgs.msg
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy import qos
from rclpy.node import Node
from sensor_msgs.msg import Joy


class AutoControlException(Exception):
    pass


def get_interface_type(type_name: str, interface_type: str) -> typing.Any:
    split = type_name.split("/")
    if len(split) != 3:
        raise AutoControlException("Invalid type_name '{}'".format(type_name))
    package = split[0]
    interface = split[1]
    message = split[2]
    if interface != interface_type:
        raise AutoControlException(
            "Cannot use interface of type '{}' for an '{}'".format(
                interface, interface_type
            )
        )

    mod = importlib.import_module(package + "." + interface_type)
    return getattr(mod, message)


def set_member(msg: typing.Any, member: str, value: typing.Any) -> None:
    ml = member.split("-")
    if len(ml) < 1:
        return
    target = msg
    for i in ml[:-1]:
        target = getattr(target, i)
    setattr(target, ml[-1], value)


class AutoControl(Node):
    """Base class for autonomous control"""

    def __init__(self, heard_tolerance: float = 0.1):
        super().__init__(
            "auto_control",
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )

        # For safety, set a "last heard" value for deadman
        self.init = False
        self.active = False
        self.deadman = -1
        self.last_heard = None
        self.heard_tolerance = heard_tolerance  # seconds

        # Don't subscribe until everything has been initialized.
        qos_profile = qos.QoSProfile(
            history=qos.QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            durability=qos.QoSDurabilityPolicy.VOLATILE,
        )

        # Subscribe to the auto control init
        self.init_sub = self.create_subscription(
            std_msgs.msg.Int8, "auto_init", self.init_callback, qos_profile
        )

        # Subscribe to joy_teleop for the deadman switch
        self.joy_sub = self.create_subscription(
            Joy, "joy", self.joy_callback, qos_profile
        )

        # Create publisher on drive
        self.pub = self.create_publisher(AckermannDriveStamped, "drive", qos_profile)

        # Create timer to publish messages
        timer_period = 1.0 / 100  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def init_callback(self, msg: std_msgs.msg.Int8) -> None:
        self.deadman = msg.data
        self.init = True

    def joy_callback(self, joy_state: Joy) -> None:
        if self.init:
            self.active = joy_state.buttons[self.deadman] == 1
        else:
            self.active = False
        self.last_heard = 1e-9 * self.get_clock().now().nanoseconds

    def timer_callback(self) -> None:
        if (self.last_heard is not None) and (self.active):
            # get the latest command
            t_now = 1e-9 * self.get_clock().now().nanoseconds
            if t_now - self.last_heard < self.heard_tolerance:
                self.pub.publish(self.get_control_command())
                # self.get_logger().info("Publishing control command")
        else:
            # publish null command
            # if not self.active:
            #     self.get_logger().info("Not active")
            # if self.last_heard is None:
            #     self.get_logger().info("No last heard")
            # self.get_logger().info("Publishing null command")
            self.pub.publish(self.get_null_command())

    def get_control_command(self) -> AckermannDriveStamped:
        raise NotImplementedError

    def get_null_command(self) -> AckermannDriveStamped:
        """Returns an empty and stamped ackermann drive message"""
        msg = AckermannDriveStamped()
        set_member(msg.header, "stamp", self.get_clock().now().to_msg())
        return msg
