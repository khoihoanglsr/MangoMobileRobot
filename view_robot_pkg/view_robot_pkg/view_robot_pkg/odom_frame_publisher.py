import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomFramePublisher(Node):
    def __init__(self):
        super().__init__('odom_frame_publisher')
        
        # 1. Khai báo tham số để sau này chỉnh trong file launch cho tiện
        self.declare_parameter('input_topic', '/odom/unfiltered')
        self.declare_parameter('output_topic', '/odom/fixed')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('child_frame_id', 'base_footprint')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value

        # 2. Tạo Subscriber (Nghe từ Arduino)
        self.subscription = self.create_subscription(
            Odometry,
            input_topic,
            self.listener_callback,
            10)

        # 3. Tạo Publisher (Gửi cho EKF)
        self.publisher = self.create_publisher(Odometry, output_topic, 10)
        
        self.get_logger().info(f'Odom Frame Fixer started: {input_topic} -> {output_topic}')

    def listener_callback(self, msg):
        # 4. "Vá" Frame ID vào tin nhắn
        # Lấy tham số hiện tại
        frame_id = self.get_parameter('frame_id').value
        child_frame_id = self.get_parameter('child_frame_id').value

        # Gán vào msg
        msg.header.frame_id = frame_id
        msg.child_frame_id = child_frame_id

        # 5. Gửi đi ngay lập tức
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomFramePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()