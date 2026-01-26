from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QColor, QMouseEvent
from PyQt6.QtCore import pyqtSignal, QPointF, Qt

class MapWidget(QWidget):
    goal_selected = pyqtSignal(float, float, float)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(400, 400)
        self.goal_point = None

    def mousePressEvent(self, event: QMouseEvent):
        if event.button() == Qt.MouseButton.LeftButton:
            pos = event.position()
            x = pos.x()
            y = pos.y()
            yaw = 0.0
            self.goal_point = QPointF(x, y)
            self.goal_selected.emit(x, y, yaw)
            self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.fillRect(self.rect(), QColor(30, 30, 30))

        # Grid giả
        painter.setPen(QColor(80, 80, 80))
        step = 40
        for x in range(0, self.width(), step):
            painter.drawLine(x, 0, x, self.height())
        for y in range(0, self.height(), step):
            painter.drawLine(0, y, self.width(), y)

        # Goal
        if self.goal_point:
            painter.setBrush(QColor(255, 0, 0))
            painter.drawEllipse(self.goal_point, 6, 6)
