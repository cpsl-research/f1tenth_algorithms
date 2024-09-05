import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import LaserScan


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg:LaserScan):
        #--variable definition--
        a_min=(msg.angle_min)
        a_max=(msg.angle_max)
        a_incr=(msg.angle_increment)
        distance_list = msg.ranges
        list_length = len(distance_list)

        filtered_array = np.array([30,500]) #creates the filtered array so values can later be appended
        #these 'dummy values' will be filtered out (couldn't figure out how to append to empty array)

        print('angle maximum',a_max) #here when testing - make sure doesn't exceed a_max

        #--filtered array from coordinates--
        for i in range (list_length):
            point_angle = a_min + (i * a_incr)
            point_angle = np.rad2deg(point_angle)
            point_distance = distance_list[i]    
            point_coords = np.array([point_angle,point_distance])
            print(point_coords)
            close_coords = filter_closeness(point_coords)
            if close_coords is not None:
                filtered_array = np.append(filtered_array,close_coords,axis=0)
        print('filtered array:',filtered_array)
        self.get_logger().info('recieved data')


def filter_closeness(point_coords):
    angle = point_coords[0]
    distance = point_coords[1]
    if angle <20:
        if angle > -20:
            if distance < 2:
                print('stop')
                return point_coords
    else:
        return np.array([30,500]) #'fake' data that gets filtered out

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    rclpy.shutdown()


if __name__ == '__main__':
    main()