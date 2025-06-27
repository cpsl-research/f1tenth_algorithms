import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from avstack_bridge.tracks import TrackBridge
from avstack_msgs.msg import BoxTrack3DArray
from rclpy import qos
from std_msgs.msg import String

from avstack_msgs.msg import BoxTrack3D

from .base import AutoControl, AutoControlException, set_member


class Follower:
    def __init__(self):
        self.half_angle_front = np.pi / 3.0
        self.max_range = 30.0
        self.reference_point = np.array([0, 0, 0])
        self.reference_ID = None

    def reset(self):
        pass

    def set_reference_point_from_tracks(self, tracks):
        self.reference_point = np.inf * np.ones(3)
        self.reference_ID = None
        self.reference_index = None

        # Get all relevant tracks
        for i_track, track in enumerate(tracks):
            if track.position.norm() < self.max_range:
                # only in front of us
                if track.position[0] > 0:
                    # only within a certain angle
                    if np.arctan2(track.position[1], track.position[0]) < self.half_angle_front:
                        if track.position.norm() < np.linalg.norm(self.reference_point):
                            self.reference_point = track.position.x
                            self.reference_ID = track.ID
                            self.reference_index = i_track

    def speed_and_steering_from_reference(self, max_speed=0.5):
        speed = self._speed_from_distance(self.reference_point[0], max_speed=max_speed)
        steering = self._steering_from_reference(self.reference_point)
        return speed, steering
    
    @staticmethod
    def _speed_from_distance(distance, max_speed):
        if distance < 0.5:
            return 0.0
        elif distance < 10.0:
            return 0.5 * max_speed
        else:
            return 1.0 * max_speed
    
    @staticmethod
    def _steering_from_reference(ref, max_steering: float=0.2):
        # Compute angle to reference point in 2D (x-y plane)
        radian_fraction = np.arctan2(ref[1], ref[0]) / (np.pi/2)
        steering = min(max_steering, abs(radian_fraction))
        steeering = -steering * np.sign(radian_fraction)

        return steeering


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

        # publish the reference point to get to
        self.publisher_reference = self.create_publisher(
            BoxTrack3D, "reference", qos_profile=qos_profile
        )

    def model_init_callback(self, init_msg: String) -> None:
        if init_msg.data == "reset":
            self.get_logger().info("Calling reset on follower")
            self.model.reset()

    def tracks_callback(self, tracks_msg: BoxTrack3DArray):
        # convert from ros types to avstack types
        tracks_avstack = TrackBridge.tracks_to_avstack(tracks_msg)

        # run the follower code to set the new reference point
        self.model.set_reference_point_from_tracks(tracks_avstack)

        # publish the selected track as reference, if one exists
        if self.model.reference_ID is not None:
            self.publisher_reference.publish(tracks_msg.tracks[self.model.reference_index])

    def get_control_command(self) -> AckermannDriveStamped:
        # preallocate the message
        drive = AckermannDrive()
        set_member(drive, "steering_angle_velocity", 0.0)
        set_member(drive, "acceleration", 0.0)
        set_member(drive, "jerk", 0.0)
        set_member(drive, "steering_angle", 0.0)
        set_member(drive, "speed", 0.0)

        # debug statement on the reference point
        self.get_logger().info("Reference point: {}".format(self.model.reference_point))

        # here take the reference point and set the speed and heading
        speed, steering_angle = self.model.speed_and_steering_from_reference()

        # debug statement on the speed and steering
        self.get_logger().info("Speed: {}, Steering: {}".format(speed, steering_angle))

        # set the members of the drive message
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
