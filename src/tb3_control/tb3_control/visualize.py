#!/usr/bin/env python3
import math
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from autunomous_module import calculate_safe_zone
from utils.Robot import Robot

class DistanceVisualizer(Node):
    def __init__(self):
        super().__init__('distance_visualizer')

        self.robot = Robot(-2, 0, -90)

        self.lidar_distances = []
        self.lidar_angles = []
        self.waypoints = [-2, -5]
        self.psi_error = 0
        
        self.create_subscription(LaserScan, '/scan', self.laser_scan_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)

        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)

        threading.Thread(target=plt.show, daemon=True).start()


    
    def odom_callback(self, msg: Odometry):
        self.robot.update_from_odom(msg)
        self.pose_received = True

    def pose_callback(self, msg: PoseStamped):
        self.robot.update_from_pose(msg)
        self.pose_received = True


    def laser_scan_callback(self, data : LaserScan):
        self.lidar_distances = np.flip(data.ranges)
        self.lidar_angles = np.linspace(0, 2 * np.pi, len(self.lidar_distances))


    def update_plot(self, frame):
        self.ax.clear()
        self.ax.set_title('Distance Data in Polar Coordinates', va='bottom')
        self.ax.set_ylim(0, 10)
        self.ax.set_theta_zero_location("N")
        self.ax.set_theta_direction(-1)

        if len(self.lidar_distances) > 0:
            self.ax.scatter(self.lidar_angles, self.lidar_distances, color='blue', s=2)
            self.ax.fill(np.linspace(0, 2 * np.pi, 360), calculate_safe_zone(self.lidar_angles, self.lidar_distances), color=[0, 0, 1, 0.2])

        if self.waypoints:
            dx = self.waypoints[0] - self.robot.x
            dy = self.waypoints[1] - self.robot.y

            print(dx, dy)
            angle = self.normalize_radian(np.arctan2(dy, dx) - self.robot.get_theta_rad())
            print(np.rad2deg(angle))
            distance = np.sqrt(dx**2 + dy**2)
            self.ax.scatter(-angle, distance, color='red', label='Waypoint')

        self.ax.plot([0, np.radians(self.psi_error)], [0, 15], color='green', label='Waypoint Angle')
        self.ax.legend()
        self.ax.grid(True)


    def normalize_radian(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    visualizer = DistanceVisualizer()
    
    spin_thread = threading.Thread(target=rclpy.spin, args=(visualizer,), daemon=True)
    spin_thread.start()

    plt.show()
    visualizer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
