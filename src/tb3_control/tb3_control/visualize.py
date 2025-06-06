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

class DistanceVisualizer(Node):
    def __init__(self):
        super().__init__('distance_visualizer')

        self.robot = Robot(0, 0, 0)

        self.lidar_distances = []
        self.lidar_angles = []

        self.laserscan_data = None
        self.waypoints = [0, 0.0]
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

        distance_threshold = 0.1          # 10cm 이내

        if distance < distance_threshold:
            # 도착 시 정지
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        else:
            cmd.linear.x = 0.1
            cmd.angular.z = float(max(min(0.5 * angle, 0.5), -0.5))


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
            dx = self.waypoints[0] - self.robot.x
            dy = self.waypoints[1] - self.robot.y

            angle = self.normalize_radian(np.arctan2(dy, dx) - self.robot.get_theta_rad())
            distance = np.sqrt(dx**2 + dy**2)

            if goal_check(self.robot, laserscan_map(self.laserscan_data), distance, np.rad2deg(angle)):
                final_angle_d = angle
            else:
                final_angle_d = np.deg2rad(pathplan(self.robot, self.laserscan_data, self.waypoints[0], self.waypoints[1])[0])

            self.ax.plot([0, final_angle_d], [0, distance], color='green', label='Waypoint Angle')
            self.ax.scatter(angle, distance, color='green', label='Waypoint')

            print(f"[로봇 위치] x: {self.robot.x:.2f}, y: {self.robot.y:.2f}, θ: {math.degrees(self.robot.theta):.1f}°")
            print(f"[목표 위치] x: {self.waypoints[0]}, y: {self.waypoints[1]}")
            print(f"[거리] {distance:.2f} m, [방향오차] {math.degrees(angle):.1f}°")
            print("[목표 각도 명령]", np.rad2deg(final_angle_d))

            # 🚀 이동 명령 발행
            self.drive_to_waypoint(final_angle_d, distance)

        self.ax.legend()
        self.ax.grid(True)

    def normalize_radian(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi

def main(args=None):
    rclpy.init(args=args)
    visualizer = DistanceVisualizer()

    # Spin ROS2 in separate thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(visualizer,), daemon=True)
    spin_thread.start()

    # GUI main loop
    plt.show()

    visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
