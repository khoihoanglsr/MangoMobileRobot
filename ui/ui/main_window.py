from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QPushButton,
    QVBoxLayout, QLabel
)
from ui.map_widget import MapWidget
from core.goal_interface import GoalSender

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mobile Robot UI - PyQt6")

        self.goal_sender = GoalSender()
        self.current_goal = None

        self.map_widget = MapWidget()
        self.status_label = QLabel("Click on map to set goal")

        self.send_button = QPushButton("Send Goal")
        self.send_button.setEnabled(False)

        layout = QVBoxLayout()
        layout.addWidget(self.map_widget)
        layout.addWidget(self.status_label)
        layout.addWidget(self.send_button)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.map_widget.goal_selected.connect(self.on_goal_selected)
        self.send_button.clicked.connect(self.send_goal)

    def on_goal_selected(self, x, y, yaw):
        self.current_goal = (x, y, yaw)
        self.status_label.setText(
            f"Goal: x={x:.1f}, y={y:.1f}, yaw={yaw:.1f}"
        )
        self.send_button.setEnabled(True)

    def send_goal(self):
        if self.current_goal:
            x, y, yaw = self.current_goal
            self.goal_sender.send_goal(x, y, yaw)
