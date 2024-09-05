#for driving
import rclpy
import ackermann_msgs.msg
import time

from .base import set_member, AutoControl, AutoControlException

#for sensing
import numpy as np

from sensor_msgs.msg import LaserScan


class RoombaControl(AutoControl):

    def __init__(self):
        super().__init__()
        self.is_path_blocked = True

    def get_control_command(self,start_time:float = time.time(),backing_status = 0) -> ackermann_msgs.msg.AckermannDriveStamped:
        self.start_time = start_time
        self.backing_status = backing_status
        # get ackermann command
        drive = ackermann_msgs.msg.AckermannDrive()
        set_member(drive, "steering_angle_velocity", 0.0)
        set_member(drive, "acceleration", 0.0)
        set_member(drive, "jerk", 0.0)

        #sets angle and speed based on lidar scan
        if self.is_path_blocked:
            #--turn in increments of 15 deg
            set_member(drive, "steering_angle", 0.261799) #15-degree turn
            set_member(drive, "speed", self.get_speed(backing_status)) #go backwards
            self.backing_status = 1
        else:
            #--keep driving straight
            set_member(drive, "steering_angle", 0.0)
            set_member(drive, "speed", 0.5)

        # package in stamped message
        
        msg = ackermann_msgs.msg.AckermannDriveStamped()
        set_member(msg.header, "stamp", self.get_clock().now().to_msg())
        set_member(msg, "drive", drive)
        
        if self.backing_status == 1:
            if time.time() >= self.start_time + 1.0:
                self.backing_status = self.backing_status - 1
                self.start_time = time.time()
        
        return msg

        
    def get_speed(backing_status):
        speed = 0.0
        if backing_status == 0:
            speed = 0.5
        elif backing_status == 1:
            speed = -0.25
        return speed
    
    
    def receive_lidar(self, msg: LaserScan):
        #--variable definition--
        a_min=(msg.angle_min)
        a_max=(msg.angle_max)
        a_incr=(msg.angle_increment)
        distance_list = msg.ranges
        list_length = len(distance_list)

        #--filtered array from coordinates--
        self.is_path_blocked = False
        for i in range (list_length):
            point_angle = a_min + (i * a_incr)
            point_angle = np.rad2deg(point_angle)
            point_distance = distance_list[i]    
            point_coords = np.array([point_angle, point_distance])
            self.is_path_blocked = self.is_path_blocked or is_path_blocked(point_coords)

#in case car backs into a wall
def check_stuck(): #executes slow forward drive/turn if front is blocked for >5 sec
    start_time=time.time()
    while True:
        current_time = time.time()
        elapsed = start_time-current_time


def is_path_blocked(point_coords):
    angle = point_coords[0]
    distance = point_coords[1]
    if angle < 20:
        if angle > -20:
            if distance < 2:
                return True
    return False


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
