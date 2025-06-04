import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math


from utils.Robot import Robot

class WaypointFollower(Node):
    def __init__(self):
        super().__init__('waypoint_follower')

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_subscription(Odometry, '/odom', self.odom_callback, 10)  # Odom 기반 위치
        # self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)  # PoseStamped 기반 위치

        self.timer = self.create_timer(0.05, self.timer_callback)

        # Robot pose initialization
        self.robot = Robot()

        # Waypoints [(x, y), ...]
        self.waypoints = [
            (3.0, 0.0),
            (3.0, 3.0),
            (0.0, 3.0),
            (0.0, 0.0)
        ]
        self.current_index = 0
        self.reached_all = False

        # Parameters
        self.linear_speed = 0.1
        self.angular_speed = 0.5
        self.dist_threshold = 0.1
        self.angle_threshold = math.radians(10)

        self.get_logger().info("Waypoint follower started.")

    def odom_callback(self, msg: Odometry):
        self.robot.update_from_odom(msg)
        self.pose_received = True

    def pose_callback(self, msg: PoseStamped):
        self.robot.update_from_pose(msg)
        self.pose_received = True

    def timer_callback(self):
        twist = Twist()

        if not self.pose_received or self.reached_all:
            self.cmd_pub.publish(twist)
            return

        # 현재 목표 웨이포인트
        goal_x, goal_y = self.waypoints[self.current_index]

        # 거리 및 방향 계산
        dx = goal_x - self.robot.x
        dy = goal_y - self.robot.y
        distance = math.hypot(dx, dy)
        target_theta = math.atan2(dy, dx)
        angle_error = self.normalize_angle(target_theta - self.robot.get_theta_rad())

        # 도착 처리
        if distance < self.dist_threshold:
            self.get_logger().info(f"Reached waypoint {self.current_index + 1}/{len(self.waypoints)}")
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self.reached_all = True
                self.get_logger().info("All waypoints reached. Stopping.")
            self.cmd_pub.publish(twist)
            return

        # 회전 정렬 먼저 수행
        if abs(angle_error) > self.angle_threshold:
            twist.angular.z = self.angular_speed if angle_error > 0 else -self.angular_speed
            twist.linear.x = 0.0
        else:
            twist.linear.x = self.linear_speed
            twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

    def normalize_angle(self, angle):
        return (angle + math.pi) % (2 * math.pi) - math.pi


def main(args=None):
    rclpy.init(args=args)
    node = WaypointFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop = Twist()
        node.cmd_pub.publish(stop)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
