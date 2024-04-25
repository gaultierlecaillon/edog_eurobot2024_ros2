#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from example_interfaces.msg import String
from robot_interfaces.msg import Position
from std_msgs.msg import Bool
import numpy
import math
import json
import time

# Led
from random import randint


class LidarFilter(Node):
    x_ = 0  # Current robot x position
    y_ = 0  # Current robot y position
    r_ = 0  # Current robot r target_angle

    def __init__(self, max_distance, min_distance, emergency_distance):
        super().__init__("lidar_filter")
        self.subscriber_ = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.emergency_distance = emergency_distance
        self.max_distance = max_distance
        self.min_distance = min_distance

        # Publish filtered ranges
        self.filter_scan_publisher_ = self.create_publisher(LaserScan, "filter_scan_topic", 10)
        self.emergency_stop_publisher_ = self.create_publisher(Bool, "emergency_stop_topic", 10)

        ''' Publisher '''
        if not hasattr(self, 'voice_publisher'):
            self.voice_publisher = self.create_publisher(String, "voice_topic", 10)

        ''' Subscribers '''
        self.create_subscription(
            Position,
            "robot_position",
            self.robot_position_callback,
            10)

        self.get_logger().info('LidarFilter node has started')

    def robot_position_callback(self, msg):
        #self.get_logger().info(f"x: {msg.x}, y: {msg.y}, r: {msg.r}")
        self.x_ = msg.x
        self.y_ = msg.y
        self.r_ = msg.r

    def scan_callback(self, msg):
        ranges_list = msg.ranges

        # Filter ranges based on max_distance
        filtered_ranges = [min(r, self.max_distance) for r in ranges_list]
        filtered_ranges = [max(r, self.min_distance) for r in filtered_ranges]

        angle_dict = {}
        for index, distance in enumerate(filtered_ranges):
            if self.min_distance < distance < self.max_distance:
                index_offset = (index + 900) % 1800  # 900 is the offset
                angle = int(360 - index_offset / 5)  # because 1800tic/360°=5

                if angle not in angle_dict:
                    angle_dict[angle] = {'count': 1, 'total_distance': distance}
                else:
                    angle_dict[angle]['count'] += 1
                    angle_dict[angle]['total_distance'] += distance

                # Calculate the average distance for the current angle
                angle_dict[angle]['average_distance'] = round(
                    angle_dict[angle]['total_distance'] / angle_dict[angle]['count'], 2)

                '''
                print(
                    len(ranges_list),
                    "tic:", index,
                    "| distance", round(distance,2),
                    '| angle:', angle,
                    '| count:', angle_dict[angle]['count'],
                    '| average_distance:', angle_dict[angle]['average_distance']
                )
                '''

        # Initialize angle_ranges list with the same length as ranges_list
        angle_ranges = [0] * len(ranges_list)

        # Fill angle_ranges with the average distances from angle_dict
        for index, _ in enumerate(angle_ranges):
            index_offset = (index + 900) % 1800
            angle = int(360 - index_offset / 5)

            if angle in angle_dict:
                angle_ranges[index] = angle_dict[angle]['average_distance']

        self.check_emergency_stop(angle_ranges)

        '''
        filtered_scan = LaserScan(
            header=msg.header,
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            time_increment=msg.time_increment,
            scan_time=msg.scan_time,
            range_min=self.min_distance,
            range_max=self.max_distance,
            ranges=angle_ranges,
            intensities=msg.intensities)
        self.filter_scan_publisher_.publish(filtered_scan)
        '''

    def check_emergency_stop(self, angle_ranges):
        emergency_stop_msg = Bool()
        emergency_stop_msg.data = False
        for index, distance in enumerate(angle_ranges):            
            if self.max_distance > distance > self.min_distance:
                index_offset = (index + 900) % 1800
                angle = int(2*360 - index_offset / 5)
                if angle > 180:
                    angle = -(360 - angle)

                # Convert polar coordinates to Cartesian coordinates
                angle_rad = numpy.radians(angle)
                dist_x = distance * numpy.cos(angle_rad)  # in m
                dist_y = distance * numpy.sin(angle_rad)  # in m
                
                x_obstacle, y_obstacle = self.calculate_obstacle_position(distance, angle)
                
                
                # Transform local obstacle coordinates to global coordinates
                
                #x_obstacle, y_obstacle = self.calculate_obstacle_position(self.x_, self.y_, dist_x, dist_y, angle_rad)
                #x_obstacle = round(dist_x * 1000 * math.cos(angle_rad) - dist_y * 1000 * math.sin(angle_rad) + self.x_)
                #y_obstacle = round(dist_x * 1000 * math.sin(angle_rad) + dist_y * 1000 * math.cos(angle_rad) + self.y_)

                #x_obstacle = self.x_ + (dist_x*1000) * numpy.cos(angle_rad) 
                #y_obstacle = self.y_ + (dist_x*1000) * numpy.sin(angle_rad)           
                self.get_logger().info(f"👮👮👮 Obstacle ! self.x_:={self.x_}m, self.y_:={self.y_}m, angle:={angle}, real_angle:={self.r_ - angle}, dist_x:={round(dist_x,2)}m, dist_y={round(dist_y,2)}m; Ostacle Position ({round(x_obstacle)}, {round(y_obstacle)})")
                self.print_robot_infos()

                if self.min_distance < dist_x < self.emergency_distance \
                        and -0.4 < dist_y < 0.4 \
                        and 200 < x_obstacle < 2800 \
                        and 200 < y_obstacle < 1800:
                    self.get_logger().info(f"👮 Obstacle ! dist_x:={round(dist_x,2)}m, dist_y={round(dist_y,2)}m; Ostacle Position ({round(x_obstacle)}, {round(y_obstacle)})")
                    emergency_stop_msg.data = True
                    self.emergency_stop_publisher_.publish(emergency_stop_msg)                    
                    return

        self.emergency_stop_publisher_.publish(emergency_stop_msg)
    
    def print_robot_infos(self):
        self.get_logger().info(
            f"\033[95m[Robot Infos] x:{self.x_}, y:{self.y_}, r:{self.r_}\033[0m")

    def calculate_obstacle_position(self, distance, angle):
        # Convert observer's orientation from degrees to radians
        ar_rad = math.radians(self.r_)
        
        # Calculate the angle from the observer to the obstacle in radians
        angle_rad = math.radians(angle)
        total_angle = ar_rad - angle_rad
        
        self.get_logger().info(f"distance={distance*1000}, angle={angle}, total_angle (rad)={total_angle}, self.x_={self.x_}, self.y_={self.y_}, self.r_={self.r_}")
        # Calculate the Cartesian coordinates of the obstacle
        ox = self.x_ + (distance*1000) * math.cos(total_angle)
        oy = self.y_ + (distance*1000) * math.sin(total_angle)
        
        return round(ox), round(oy)

def main(args=None):
    rclpy.init(args=args)
    with open('/home/edog/ros2_ws/src/control_package/resource/robot_config.json') as file:
            config = json.load(file)
    node = LidarFilter(config['robot']['emergency_distance'], config['robot']['min_distance'], config['robot']['max_distance'])

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
