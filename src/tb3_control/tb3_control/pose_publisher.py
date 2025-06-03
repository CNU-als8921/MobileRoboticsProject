import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from geometry_msgs.msg import PoseStamped, Point
from builtin_interfaces.msg import Time as TimeMsg

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')

        # TF 버퍼 및 리스너 설정
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # PoseStamped 메시지 퍼블리셔 생성
        self.pose_pub = self.create_publisher(PoseStamped, '/robot_pose', 10)

        # 주기적으로 콜백 실행 (0.5초 간격)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        try:
            # map → base_link 변환 요청
            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time())

            # PoseStamped 메시지 생성 및 변환 데이터 설정
            pose = PoseStamped()
            pose.header.stamp = trans.header.stamp
            pose.header.frame_id = 'map'

            # Vector3 → Point 변환
            pose.pose.position = Point(
                x=trans.transform.translation.x,
                y=trans.transform.translation.y,
                z=trans.transform.translation.z
            )

            # orientation 그대로 복사
            pose.pose.orientation = trans.transform.rotation

            # 메시지 publish
            self.pose_pub.publish(pose)
            self.get_logger().info(f"Published pose: {pose.pose.position}")

        except Exception as e:
            self.get_logger().warn(f"Could not transform: {e}")

def main():
    rclpy.init()
    node = PosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
