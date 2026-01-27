import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFBroadcaster(Node):
    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.VOLATILE

        self.subscription = self.create_subscription(
            Odometry,
            'odomfromSTM32',
            self.odom_callback,
            qos
        )

        # Publisher chuẩn cho Nav2
        self.odom_pub = self.create_publisher(Odometry, 'odom', qos)

        self.tf_broadcaster = TransformBroadcaster(self)

        # state cuối
        self.x = 0.0
        self.y = 0.0
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        # publish TF đều đặn
        self.timer = self.create_timer(0.1, self.publish_tf)  # 50 Hz

        self.get_logger().info("Odom TF Broadcaster started (odom → base_footprint) + republish /odom")

    def odom_callback(self, msg: Odometry):
        # Lưu pose
        self.x = float(msg.pose.pose.position.x)
        self.y = float(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation
        self.qx = float(q.x)
        self.qy = float(q.y)
        self.qz = float(q.z)
        self.qw = float(q.w)

        # Republish odom ra topic /odom cho Nav2
        out = Odometry()
        out.header.stamp = self.get_clock().now().to_msg()   # hoặc msg.header.stamp nếu STM32 stamp chuẩn
        out.header.frame_id = 'odom'
        out.child_frame_id = 'base_footprint'

        out.pose = msg.pose
        out.twist = msg.twist

        self.odom_pub.publish(out)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'

        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = self.qx
        t.transform.rotation.y = self.qy
        t.transform.rotation.z = self.qz
        t.transform.rotation.w = self.qw

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
