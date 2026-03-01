"""
Visualizes the car's path and surrounding features on a 2D canvas.

Coordinate conventions:
  World space:  +X = FORWARD, +Y = LEFT  (meters)
  Screen space: +X = RIGHT,   +Y = DOWN  (pixels)
"""

from PyQt6.QtWidgets import QWidget, QSizePolicy
from PyQt6.QtGui import QPainter, QPen, QBrush
from PyQt6.QtCore import Qt, QTimer

from .node import AP1ConsoleNode


# ===== Colors =====
WHITE = Qt.GlobalColor.white
DIM = Qt.GlobalColor.darkGray
RED = Qt.GlobalColor.red
GREEN = Qt.GlobalColor.green
YELLOW = Qt.GlobalColor.yellow
PURPLE = Qt.GlobalColor.darkMagenta
BLUE = Qt.GlobalColor.blue
BLACK = Qt.GlobalColor.black


# ===== Canvas Constants =====
CANVAS_W = 60
CANVAS_H = 60
ORIGIN_X = CANVAS_W // 2
ORIGIN_Y = CANVAS_H - 1

WORLD_RANGE_X = 40
WORLD_RANGE_Y = 40

DOT_SIZE = 6


# ===== Helper Point =====
class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y


def canvas_to_screen(cx: float, cy: float, screen_w: int, screen_h: int):
    scale_x = screen_w / CANVAS_W
    scale_y = screen_h / CANVAS_H

    sx = (ORIGIN_X + cx) * scale_x
    sy = (ORIGIN_Y + cy) * scale_y
    return int(sx), int(sy)


# ===== Main Widget =====
class PathCanvas(QWidget):
    REFRESH_RATE = 10  # Hz

    def __init__(self, node: AP1ConsoleNode, parent=None):
        super().__init__(parent)
        self.node = node

        self.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.setMinimumSize(300, 300)
        self.setStyleSheet("background: black;")

        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000 / self.REFRESH_RATE))

        self._zoom = 1.0
        self._pan_x = 0.0
        self._pan_y = 0.0
        self._drag_start = None
        self._pan_start = None

    # ===== Coordinate Transform =====
    def world_to_screen(self, point: Point):
        effective_range_x = WORLD_RANGE_X * self._zoom
        effective_range_y = WORLD_RANGE_Y * self._zoom

        cx = -(point.y - self._pan_y) / effective_range_y * CANVAS_W
        cy = -(point.x - self._pan_x) / effective_range_x * CANVAS_H

        return canvas_to_screen(cx, cy, self.width(), self.height())

    # ===== Zoom =====
    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        factor = 1.1 if delta < 0 else 0.9
        self._zoom = max(0.1, min(10.0, self._zoom * factor))
        self.update()

    # ===== Pan =====
    def mousePressEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_start = event.position()
            self._pan_start = (self._pan_x, self._pan_y)

    def mouseMoveEvent(self, event):
        if self._drag_start is not None:
            dx = event.position().x() - self._drag_start.x()
            dy = event.position().y() - self._drag_start.y()

            w, h = self.width(), self.height()
            world_range_x = WORLD_RANGE_X * self._zoom
            world_range_y = WORLD_RANGE_Y * self._zoom

            self._pan_x = self._pan_start[0] + (dy / h) * world_range_x
            self._pan_y = self._pan_start[1] + (dx / w) * world_range_y
            self.update()

    def mouseReleaseEvent(self, event):
        if event.button() == Qt.MouseButton.LeftButton:
            self._drag_start = None

    # ===== Paint =====
    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        painter.fillRect(self.rect(), BLACK)

        self._draw_axes(painter)
        self._draw_path(painter)
        self._draw_car(painter)

        painter.end()

    # ===== Drawing Helpers =====
    def _draw_axes(self, painter: QPainter):
        pen = QPen(DIM)
        pen.setWidth(1)
        painter.setPen(pen)

        ox, oy = canvas_to_screen(0, 0, self.width(), self.height())
        painter.drawLine(0, oy, self.width(), oy)
        painter.drawLine(ox, 0, ox, self.height())

    def _draw_car(self, painter: QPainter):
        px, py = self.world_to_screen(Point(0, 0))
        size = 8

        pen = QPen(YELLOW)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        painter.drawLine(px - size, py, px + size, py)
        painter.drawLine(px, py - size, px, py + size)
        painter.drawEllipse(px - size, py - size, size * 2, size * 2)

    def _draw_path(self, painter: QPainter):
        waypoints = self.node.target_path[:]

        pen = QPen(WHITE)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        for a, b in zip(waypoints, waypoints[1:]):
            x1, y1 = self.world_to_screen(a)
            x2, y2 = self.world_to_screen(b)
            painter.drawLine(x1, y1, x2, y2)

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(GREEN))
        r = DOT_SIZE // 2

        for pt in waypoints:
            px, py = self.world_to_screen(pt)
            painter.drawEllipse(px - r, py - r, DOT_SIZE, DOT_SIZE)