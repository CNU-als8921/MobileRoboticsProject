from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math

class Robot:
    def __init__(self, x=0.0, y=0.0, theta_deg=0.0):
        self.x = x
        self.y = y
        self.theta = theta_deg

    def update_from_odom(self, odom_msg : Odometry):
        pose = odom_msg.pose.pose
        self._update_pose(pose.position.x, pose.position.y, pose.orientation)

    def update_from_pose(self, pose_msg : PoseStamped):
        pose = pose_msg.pose
        self._update_pose(pose.position.x, pose.position.y, pose.orientation)

    def _update_pose(self, x, y, q):
        self.x = x
        self.y = y

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        theta_rad = math.atan2(siny_cosp, cosy_cosp)
        self.theta = math.degrees(theta_rad)

    def get_theta_rad(self):
        return math.radians(self.theta)
