#wiggle code below
import rclpy
import ackermann_msgs.msg
import time
from .base import set_member, AutoControl, AutoControlException


class WiggleControl(AutoControl):
    #drives in square (theoretcally)

    def __init__(self, dt_state: float = 1.5, n_states: int = 4, speed: float = 1.0, angular_velocity: float = 1.0):
        super().__init__()
        self.state = 0
        self.start_time = time.time()
        self.dt_state = dt_state
        self.n_states = n_states
        self.speed = speed
        self.angular_velocity = angular_velocity

    def get_control_command(self) -> ackermann_msgs.msg.AckermannDriveStamped:
        # get ackermann command
        drive = ackermann_msgs.msg.AckermannDrive()

        # -- lateral
        set_member(drive, "steering_angle", self.get_steering_angle(self.state))
        set_member(drive, "steering_angle_velocity", float(self.angular_velocity))

        # -- longitudinal
        set_member(drive, "speed", float(self.speed))
        set_member(drive, "acceleration", 0.0)
        set_member(drive, "jerk", 0.0)

        # package in stamped message
        msg = ackermann_msgs.msg.AckermannDriveStamped()
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
    def get_steering_angle(state: int) -> float:
        if state in [0, 2]:
            ang = 0.0
        elif state == 1:
            ang = 0.78539  # turn left
        elif state == 3:
            ang = -0.78539  # turn right
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

