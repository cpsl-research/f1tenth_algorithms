import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from avstack_bridge.tracks import TrackBridge
from avstack_msgs.msg import BoxTrack3DArray
from rclpy import qos
from std_msgs.msg import String

from .base import AutoControl, AutoControlException, set_member


class Follower:
    def __init__(self):
        self.reference_point = np.array([0, 0, 0])

    def reset(self):
        pass

    def set_reference_point_from_tracks(self, tracks):
        # TODO
        # each track state is [x, y, z, h, w, l, vx, vy, vz]
        track_states = [track.x for track in tracks.data]
        track_IDs = [track.ID for track in tracks.data]

        # set the reference point somehow
        reference_point = np.array([0, 0, 0])

        return reference_point

    def speed_and_steering_from_reference(self):
        # TODO - use reference point to get speed and steering angle
        speed = 0.0
        steering_angle = 0.0
        return speed, steering_angle


class FollowerControl(AutoControl):
    def __init__(
        self,
        range_close: float = 3.0,
        drive_speed: float = 2.0,
    ):
        super().__init__()
        self.range_close = range_close
        self.drive_speed = drive_speed
        self.model = Follower()
        self.get_logger().info("Initialized FollowerControl")

        qos_profile = qos.QoSProfile(
            history=qos.QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=qos.QoSReliabilityPolicy.RELIABLE,
            durability=qos.QoSDurabilityPolicy.VOLATILE,
        )

        # subscribe to the tracks
        self.subscriber_tracks = self.create_subscription(
            BoxTrack3DArray,
            "tracks_3d",
            self.tracks_callback,
            qos_profile=qos_profile,
        )

    def init_callback(self, init_msg: String) -> None:
        if init_msg.data == "reset":
            self.get_logger().info("Calling reset on follower")
            self.model.reset()

    def tracks_callback(self, tracks_msg: BoxTrack3DArray):
        # convert from ros types to avstack types
        tracks_avstack = TrackBridge.tracks_to_avstack(tracks_msg)

        # run the follower code to set the new reference point
        self.model.set_reference_point_from_tracks(tracks_avstack)

    def get_control_command(self) -> AckermannDriveStamped:
        # preallocate the message
        drive = AckermannDrive()
        set_member(drive, "steering_angle_velocity", 0.0)
        set_member(drive, "acceleration", 0.0)
        set_member(drive, "jerk", 0.0)
        set_member(drive, "steering_angle", 0.0)
        set_member(drive, "speed", 0.0)

        # here take the reference point and set the speed and heading
        speed, steering_angle = self.model.speed_and_steering_from_reference()
        set_member(drive, "speed", speed)
        set_member(drive, "steering_angle", steering_angle)

        # add the header
        msg = AckermannDriveStamped()
        set_member(msg.header, "stamp", self.get_clock().now().to_msg())
        set_member(msg, "drive", drive)
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = FollowerControl()

    try:
        rclpy.spin(node)
    except AutoControlException as e:
        node.get_logger().error(e.message)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
