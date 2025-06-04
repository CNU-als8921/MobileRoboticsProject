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
        self.pose_received = False

        # Parameters
        self.linear_speed = 0.3
        self.angular_speed_max = 5.0  # Maximum angular speed (rad/s)
        self.dist_threshold = 0.1

        # PID coefficients for angular control
        self.kp = 2.0
        self.ki = 0.0
        self.kd = 0.5
        self.prev_angle_error = 0.0
        self.integral = 0.0

        self.get_logger().info("Waypoint follower started.")

    def odom_callback(self, msg: Odometry):
        self.robot.update_from_odom(msg)
        self.pose_received = True

    def pose_callback(self, msg: PoseStamped):
        self.robot.update_from_pose(msg)
        self.pose_received = True

    def pid_control(self, angle_error):
        # PID control for angular velocity
        self.integral += angle_error
        derivative = angle_error - self.prev_angle_error
        angular_velocity = self.kp * angle_error + self.ki * self.integral + self.kd * derivative
        self.prev_angle_error = angle_error

        # Limit angular velocity to the maximum angular speed
        angular_velocity = max(min(angular_velocity, self.angular_speed_max), -self.angular_speed_max)

        return angular_velocity

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
        angle_error = self.normalize_radian(target_theta - self.robot.get_theta_rad())

        # 도착 처리
        if distance < self.dist_threshold:
            self.get_logger().info(f"Reached waypoint {self.current_index + 1}/{len(self.waypoints)}")
            self.current_index += 1
            if self.current_index >= len(self.waypoints):
                self.reached_all = True
                self.get_logger().info("All waypoints reached. Stopping.")
            self.cmd_pub.publish(twist)
            return

        # PID 제어를 통해 각도 맞추기
        angular_velocity = self.pid_control(angle_error)

        # 전진 속도 고정
        twist.linear.x = self.linear_speed
        twist.angular.z = angular_velocity
        print(f"Linear Speed (v): {twist.linear.x:.2f}, Angular Speed (w): {twist.angular.z:.2f}")


        self.cmd_pub.publish(twist)

    def normalize_radian(self, angle):
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
