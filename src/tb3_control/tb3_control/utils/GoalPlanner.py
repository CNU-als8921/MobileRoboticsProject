import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import math
from Robot import Robot
from geometry_msgs.msg import PoseStamped
class GoalPlanner:
    def __init__(self, desX, desY, desTheta, robot : Robot):
        self.goal_x = desX
        self.goal_y = desY
        self.goal_theta = desTheta
        self.robot = robot

        self.K_rho = 0
        self.K_alpha = 0
        self.K_beta = 0

        self.mode = "FORWARD"

    def setGoalFromPose(self, pose_msg : PoseStamped):
        q = pose_msg.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta_rad = math.atan2(siny_cosp, cosy_cosp)

        self.goal_x = pose_msg.pose.position.x
        self.goal_y = pose_msg.pose.position.y
        self.goal_theta = math.degrees(theta_rad)

    def setGoal(self, x, y, theta_deg):
        self.goal_x = x
        self.goal_y = y
        self.goal_theta = theta_deg
        
    def setParameter(self, k_r, k_a, k_b):
        self.K_rho = k_r
        self.K_alpha = k_a
        self.K_beta = k_b

    def saturationRad(self, rad):
        return (rad + np.pi) % (2 * np.pi) - np.pi

    def calculateVelocity(self):

        dx = self.goal_x - self.robot.x  # 북쪽 차이
        dy = self.goal_y - self.robot.y  # 서쪽 차이

        path_theta = np.arctan2(dy, dx)  # (y=서쪽, x=북쪽)

        rho = np.hypot(dx, dy)
        alpha = self.saturationRad(path_theta - np.deg2rad(self.robot.theta))
        beta = self.saturationRad(np.deg2rad(self.goal_theta) - path_theta)

        v = self.K_rho * rho
        w = self.K_alpha * alpha + self.K_beta * beta

        if rho < 0.05:
            v = 0
            heading_error = self.saturationRad(np.deg2rad(self.goal_theta) - np.deg2rad(self.robot.theta))
            w = 1.0 * heading_error

        return v, w
