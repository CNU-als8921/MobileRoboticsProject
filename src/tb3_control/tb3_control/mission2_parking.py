import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from utils.Robot import Robot
from utils.GoalPlanner import GoalPlanner


class GoalNavigationNode(Node):
    def __init__(self):
        super().__init__('goal_navigation_node')

        # Robot & Goal 관련 초기화
        self.robot = Robot(0, 0, 0)  # 초기 pose
        self.planner = None  # GoalPlanner는 나중에 생성

        self.goal_x = None
        self.goal_y = None
        self.goal_theta = None  # degree 단위

        self.max_linear_velocity = 0.1   # m/s
        self.max_angular_velocity = 1  # rad/s

        self.pose_received = False

        ##### Odom #####
        # self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        # self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)

        ##### SLAM #####
        self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)
        self.create_subscription(PoseStamped, '/move_base_simple/goal', self.goal_callback, 10)


        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info("Goal Navigation Node Started.")

    def pose_callback(self, msg: PoseStamped):
        self.robot.update_from_pose(msg)
        self.pose_received = True

    def odom_callback(self, msg: Odometry):
        self.robot.update_from_odom(msg)
        self.pose_received = True

    def goal_callback(self, msg: PoseStamped):
        if self.planner is None:
            self.planner = GoalPlanner(0, 0, 0, self.robot)
            self.planner.set_goal_from_pose(msg)
            self.planner.set_parameter(0.3, 0.8, -0.15)
        else:
            self.planner.set_goal_from_pose(msg)

        self.get_logger().info(f"New goal received: x={self.planner.goal_x:.2f}, y={self.planner.goal_y:.2f}, theta={self.planner.goal_theta:.2f} deg")

    def timer_callback(self):
        twist = Twist()

        if not self.pose_received or self.planner is None:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            if not self.pose_received:
                self.get_logger().warn("Waiting for first pose... Publishing zero velocity.")
            else:
                self.get_logger().info("Waiting for goal... Publishing zero velocity.")
            return

        v, w = self.planner.calculate_velocity()
        v = self.saturation_velocity(v)
        w = self.saturation_angular_velocity(w)

        twist.linear.x = float(v)
        twist.angular.z = float(w)
        self.cmd_vel_pub.publish(twist)

        self.get_logger().info(f"x={self.robot.x:.2f}, y={self.robot.y:.2f}, theta={self.robot.theta:.2f}")
        self.get_logger().info(f"Publishing cmd_vel: v={v:.2f}, w={w:.2f}")

    def saturation_velocity(self, v):
        return max(min(v, self.max_linear_velocity), -self.max_linear_velocity)

    def saturation_angular_velocity(self, w):
        return max(min(w, self.max_angular_velocity), -self.max_angular_velocity)


def main(args=None):
    rclpy.init(args=args)
    node = GoalNavigationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    finally:
        stop_twist = Twist()
        stop_twist.linear.x = 0.0
        stop_twist.angular.z = 0.0
        node.cmd_vel_pub.publish(stop_twist)
        node.get_logger().info("Published stop command before shutdown.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
