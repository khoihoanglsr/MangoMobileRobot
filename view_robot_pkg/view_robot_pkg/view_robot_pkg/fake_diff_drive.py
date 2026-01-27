#!/usr/bin/env python3
import rclpy, math, time
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster

class FakeDiff(Node):
    def __init__(self):
        super().__init__('fake_diff_drive')
        # --- thông số ---
        self.declare_parameter('wheel_radius', 0.0425)
        self.declare_parameter('wheel_separation', 0.264)
        self.r = float(self.get_parameter('wheel_radius').value)
        self.L = float(self.get_parameter('wheel_separation').value)

        # --- biến trạng thái ---
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = time.time()
        self.theta_l = 0.0
        self.theta_r = 0.0

        # --- ROS I/O ---
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.pub_js = self.create_publisher(JointState, '/joint_states', 10)
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.02, self.update)  # 50 Hz

        self.get_logger().info('✅ fake_diff_drive started (listening to /cmd_vel)')

    def cmd_callback(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now

        # Tính toán vị trí mới
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw += self.w * dt

        # Tính toán góc quay bánh
        wl = (self.v - self.w*self.L/2.0) / self.r
        wr = (self.v + self.w*self.L/2.0) / self.r
        self.theta_l += wl * dt
        self.theta_r += wr * dt

        # --- Xuất TF odom->base_link ---
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        cy = math.cos(self.yaw * 0.5)
        sy = math.sin(self.yaw * 0.5)
        t.transform.rotation.z = sy
        t.transform.rotation.w = cy
        self.br.sendTransform(t)

        # --- Xuất joint_states ---
        js = JointState()
        js.header.stamp = t.header.stamp
        js.name = ['leftwheel_center_joint', 'rightwheel_center_joint']
        js.position = [self.theta_l, self.theta_r]
        self.pub_js.publish(js)

def main():
    rclpy.init()
    node = FakeDiff()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

