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
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry

from utils.autunomous_module import calculate_safe_zone, goal_check, laserscan_map, pathplan
from utils.Robot import Robot

class AvoidanceNode(Node):
    def __init__(self):
        super().__init__('distance_visualizer')
        self.MAX_LINEAR_SPEED = 0.4
        self.MIN_LINEAR_SPEED = -0.4
        self.MAX_ANGULER_SPEED = 1


        self.robot = Robot(0, 0, 0)

        self.lidar_distances = []
        self.lidar_angles = []

        self.laserscan_data = None
        self.waypoints = [0, 0.0]
        self.waypoints = [
            (2.0, 1.0),
            (4.0, -1.0),
            (6.0, 1.0),
            (0.0, 0.0)
        ]

        self.psi_error = 0

        # ROS2 subscriptions
        self.create_subscription(LaserScan, '/scan', self.laser_scan_callback, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)

        # ROS2 publisher for cmd_vel
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Matplotlib setup
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.ani = FuncAnimation(self.fig, self.update_plot, interval=100)

        # Start the GUI in main thread
        threading.Thread(target=plt.show, daemon=True).start()

    def odom_callback(self, msg: Odometry):
        self.robot.update_from_odom(msg)
        self.pose_received = True

    def pose_callback(self, msg: PoseStamped):
        self.robot.update_from_pose(msg)
        self.pose_received = True

    def laser_scan_callback(self, data: LaserScan):
        self.laserscan_data = data

    def drive_to_waypoint(self, angle, distance):
        cmd = Twist()

        distance_threshold = 0.25
        if distance < distance_threshold:
            self.waypoints.pop(0)
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd.angular.z = float(max(min(self.MAX_ANGULER_SPEED * angle, 0.5), -self.MAX_ANGULER_SPEED))
            min_speed = 0.05

            scaling_factor = (1.0 - min(abs(cmd.angular.z) / self.MAX_ANGULER_SPEED, 1.0)) ** 2
            cmd.linear.x = min_speed + (self.MAX_LINEAR_SPEED - min_speed) * scaling_factor

        cmd.linear.x = self.saturate(cmd.linear.x, self.MIN_LINEAR_SPEED, self.MAX_LINEAR_SPEED)
        cmd.angular.z = self.saturate(cmd.angular.z, -self.MAX_ANGULER_SPEED, self.MAX_ANGULER_SPEED)


        print(cmd.linear.x, cmd.angular.z)
        self.cmd_pub.publish(cmd)

    def update_plot(self, frame):
        self.ax.clear()
        self.ax.set_title('Distance Data in Polar Coordinates', va='bottom')
        self.ax.set_ylim(0, 10)
        self.ax.set_theta_zero_location("N")

        if self.laserscan_data:
            lidar_distances = np.array(self.laserscan_data.ranges)
            lidar_angles = np.linspace(0, 2 * np.pi, len(lidar_distances))
            self.ax.scatter(lidar_angles, lidar_distances, color='blue', s=2)
            self.ax.fill(np.linspace(0, 2 * np.pi, 360),
                         calculate_safe_zone(lidar_angles, lidar_distances),
                         color=[0, 0, 1, 0.2])
        else:
            print("Waiting for LaserScan...")
            return


        if self.waypoints:
            dx = self.waypoints[0][0] - self.robot.x
            dy = self.waypoints[0][1] - self.robot.y

            angle = self.normalize_radian(np.arctan2(dy, dx) - self.robot.get_theta_rad())
            distance = np.sqrt(dx**2 + dy**2)

            if goal_check(self.robot, laserscan_map(self.laserscan_data), distance, np.rad2deg(angle)):
                final_angle_d = angle
            else:
                final_angle_d = np.deg2rad(pathplan(self.robot, self.laserscan_data, self.waypoints[0][0], self.waypoints[0][1])[0])

            self.ax.plot([0, final_angle_d], [0, distance], color='green', label='Waypoint Angle')
            self.ax.scatter(angle, distance, color='green', label='Waypoint')

            # 🚀 이동 명령 발행
            self.drive_to_waypoint(final_angle_d, distance)

    def normalize_radian(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def saturate(self, value, min_value, max_value):
        return max(min(value, max_value), min_value)


def main(args=None):
    rclpy.init(args=args)
    avoidance_node = AvoidanceNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(avoidance_node,), daemon=True)
    spin_thread.start()

    plt.show()

    avoidance_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
