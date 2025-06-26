import math
from typing import List, Optional
import numpy as np
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive, AckermannDriveStamped
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration
from auto_py.perception.person_detection import PersonDetectionClusterer, ClusterTracker
from ..perception.base import Cluster, laser_scan_to_points
from .base import AutoControl, AutoControlException, set_member


class FollowerControl(AutoControl):
    def __init__(
        self,
        range_close: float = 3.0,
        steering_angle: float = 0.0,
        drive_speed: float = 2.0,
    ):
        super().__init__()
        self.range_close = range_close
        self.cluster_detector = PersonDetectionClusterer()
        self.steering_angle = steering_angle
        self.drive_speed = drive_speed
        self.cluster_tracker = ClusterTracker()

        # Publisher for visualization markers (cluster centers)
        self.marker_pub = self.create_publisher(Marker, '/detections_marker', 10)

        # Subscriber to LiDAR scans
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.receive_lidar,
            10
        )

    def get_control_command(self) -> AckermannDriveStamped:
        drive = AckermannDrive()
        set_member(drive, "steering_angle_velocity", 0.0)
        set_member(drive, "acceleration", 0.0)
        set_member(drive, "jerk", 0.0)
        set_member(drive, "steering_angle", self.steering_angle)
        set_member(drive, "speed", self.drive_speed)

        msg = AckermannDriveStamped()
        set_member(msg.header, "stamp", self.get_clock().now().to_msg())
        set_member(msg, "drive", drive)
        return msg

    def receive_lidar(self, msg: LaserScan):
        # Convert LaserScan to 2D points
        points = laser_scan_to_points(msg)

        # Detect clusters (people)
        clusters = self.detect_people(points)

        # Track clusters and assign persistent IDs
        tracked_clusters = self.cluster_tracker.track(clusters)

        # Build visualization marker message
        marker = Marker()
        marker.header.frame_id = "base_link"  # Adjust this to match your TF frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detections"
        marker.id = 0
        marker.type = Marker.POINTS
        marker.action = Marker.ADD

        # Set appearance
        marker.scale.x = 0.2  # point width
        marker.scale.y = 0.2  # point height
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.lifetime = Duration(sec=0)  # Show marker until overwritten

        # Add cluster centers with IDs
        for cluster in tracked_clusters:
            center = cluster.center()
            pt = Point()
            pt.x = float(center[0])
            pt.y = float(center[1])
            pt.z = 0.0
            marker.points.append(pt)

            # Add a label to visualize the cluster ID in RViz
            label = Marker()
            label.header.frame_id = "base_link"
            label.header.stamp = self.get_clock().now().to_msg()
            label.ns = "detections_ids"
            label.id = cluster.id
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = pt
            label.text = f"ID: {cluster.id}"
            label.scale.z = 0.5  # Size of the text
            label.color.a = 1.0
            label.color.r = 1.0
            label.color.g = 0.0
            label.color.b = 0.0

            self.marker_pub.publish(label)

        # Publish to RViz
        self.marker_pub.publish(marker)

    def detect_people(self, points: np.ndarray) -> List[Cluster]:
        return self.cluster_detector(points)


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
