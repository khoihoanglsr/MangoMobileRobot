import sys
import rclpy
from rclpy.node import Node

from PyQt6.QtWidgets import QApplication, QLabel, QVBoxLayout, QWidget
from PyQt6.QtCore import QTimer

class UINode(Node):
    def __init__(self):
        super().__init__('ui_node')

class MainWindow(QWidget):
    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        self.label = QLabel("PyQt6 + ROS2 is running")
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        self.setLayout(layout)

def main():
    rclpy.init()

    ros_node = UINode()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    # Timer để ROS spin mà không block Qt
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0.01))
    timer.start(10)

    app.exec()

    ros_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
