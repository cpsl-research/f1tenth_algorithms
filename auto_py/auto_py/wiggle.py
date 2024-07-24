#wiggle code below
import rclpy
import ackermann_msgs.msg
import time
from .base import set_member, AutoControl, AutoControlException


class WiggleControl(AutoControl):
    #drives in square (theoretcally)

    def __init__(self, steering_angle: float = 0.0, counter: int = 0,start_time: float = time.time()):
        super().__init__()
        self.steering_angle = steering_angle
        self.counter = counter
        self.start_time = start_time

    def get_control_command(self) -> ackermann_msgs.msg.AckermannDriveStamped:
        # get ackermann command
        drive = ackermann_msgs.msg.AckermannDrive()
        # -- lateral
        self.steering_angle = get_steering_angle(self.counter)
        set_member(drive, "steering_angle", self.steering_angle)
     
        set_member(drive, "steering_angle_velocity", self.steering_angle)
        # -- longitudinal
        set_member(drive, "speed", 1.0)
        set_member(drive, "acceleration", 0.0)
        set_member(drive, "jerk", 0.0)

        # package in stamped message
        msg = ackermann_msgs.msg.AckermannDriveStamped()
        set_member(msg.header, "stamp", self.get_clock().now().to_msg())
        set_member(msg, "drive", drive)

        #time & counter tracking
        if self.counter == 0:
            if time.time() >= self.start_time + 1.0:
                self.counter = self.counter + 1
                self.start_time = time.time()
        if self.counter == 1:
            if time.time() >= self.start_time +1.33:
                self.counter = self.counter - 1
                self.start_time = time.time()
        
        return msg

        
def get_steering_angle(count):
    ang = 0.0
    if count == 0:
        ang = 0.0
    elif count == 1:
        ang = 0.78539
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

