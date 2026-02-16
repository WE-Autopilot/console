"""
Visualizes the car's path and surrounding features on a 2D canvas.

Coordinate conventions:
  World space:  +X = FORWARD, +Y = LEFT  (meters)
  Screen space: +X = RIGHT,   +Y = DOWN  (pixels)

Transform pipeline:
  world_point
    → world_to_canvas()   scale meters → logical canvas units, flip axes
    → canvas_to_screen()  scale logical units → screen pixels
"""

from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QPen, QBrush
from PyQt6.QtCore import Qt, QTimer

from .node import AP1ConsoleNode

# || DEFAULTS
DEFAULT_FEATURE_TYPE = "stop_sign" # TEMPORARY, entities don't contain types yet so this is for now

# || Colors 
WHITE  = Qt.GlobalColor.white
DIM    = Qt.GlobalColor.darkGray
RED    = Qt.GlobalColor.red
GREEN  = Qt.GlobalColor.green
YELLOW = Qt.GlobalColor.yellow
PURPLE = Qt.GlobalColor.darkMagenta
BLUE   = Qt.GlobalColor.blue
BLACK  = Qt.GlobalColor.black

FEATURE_COLORS = {
    'stop_sign':     RED,
    'traffic_light': GREEN,
    'stop_line':     YELLOW,
    'yield_sign':    BLUE,
}

# || Canvas constants
# The logical canvas is a fixed 60×60 unit grid. The car sits at the
# bottom-center: (ORIGIN_X, ORIGIN_Y) in logical units.
CANVAS_W  = 60             # logical width  (units)
CANVAS_H  = 60             # logical height (units)
ORIGIN_X  = CANVAS_W // 2 # horizontal center — car is laterally centered
ORIGIN_Y  = CANVAS_H - 1  # near bottom edge  — car is at the bottom

# World extent visible on the canvas (meters)
WORLD_RANGE_X = 20  # meters forward/backward
WORLD_RANGE_Y = 20  # meters left/right

DOT_SIZE    = 6  # px – waypoint / feature marker diameter
TARGET_SIZE = 8  # px – target location marker diameter


# || Coordinate transforms

class Point:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y


def world_to_canvas(point: Point) -> tuple[float, float]:
    """
    Convert a world-space point to logical canvas coordinates.

    World convention:  +X = forward (up on screen), +Y = left (left on screen)
    Canvas convention: origin at bottom-center; +canvas_x = right, +canvas_y = up

    Axis mapping:
        world +X (forward) → canvas -Y (up on screen, since screen Y is flipped)
        world +Y (left)    → canvas -X (left on screen)

    Both axes are scaled so WORLD_RANGE maps to the full canvas extent.
    """
    canvas_x = -point.y / WORLD_RANGE_Y * CANVAS_W  # left/right
    canvas_y = -point.x / WORLD_RANGE_X * CANVAS_H  # forward/back
    return canvas_x, canvas_y


def canvas_to_screen(cx: float, cy: float, screen_w: int, screen_h: int) -> tuple[int, int]:
    """
    Convert logical canvas coordinates to screen pixel coordinates.

    The logical canvas is CANVAS_W × CANVAS_H units, with the car origin at
    (ORIGIN_X, ORIGIN_Y). This function:
      1. Shifts the point relative to the car origin.
      2. Scales to fill the current widget dimensions.

    Screen convention: origin top-left, +X right, +Y down.
    Note: canvas_y is already negative for "up", so adding it to ORIGIN_Y
    correctly moves points toward the top of the screen.
    """
    scale_x = screen_w / CANVAS_W
    scale_y = screen_h / CANVAS_H

    sx = (ORIGIN_X + cx) * scale_x
    sy = (ORIGIN_Y + cy) * scale_y
    return int(sx), int(sy)


def world_to_screen(point: Point, screen_w: int, screen_h: int) -> tuple[int, int]:
    """
    Full pipeline: world space → logical canvas → screen pixels.
    Use this for all drawing operations.
    """
    cx, cy = world_to_canvas(point)
    return canvas_to_screen(cx, cy, screen_w, screen_h)


# || Widget

class PathCanvas(QWidget):
    REFRESH_RATE = 10  # Hz

    def __init__(self, node: AP1ConsoleNode, parent=None):
        super().__init__(parent)
        self.node = node

        self.setMinimumSize(300, 300)
        self.setStyleSheet("background: black;")

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000 / self.REFRESH_RATE))

    #  Painting 

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        w, h = self.width(), self.height()
        painter.fillRect(0, 0, w, h, BLACK)

        self._draw_axes(painter, w, h)
        self._draw_path(painter, w, h)
        self._draw_features(painter, w, h)
        self._draw_target(painter, w, h)
        self._draw_lanes(painter, w, h)

        painter.end()

    def _draw_axes(self, painter: QPainter, w: int, h: int):
        """Draw the X (forward) and Y (left) reference axes through the car origin, with labels."""
        pen = QPen(DIM)
        pen.setWidth(1)
        painter.setPen(pen)

        ox, oy = canvas_to_screen(0, 0, w, h)
        painter.drawLine(0, oy, w, oy)  # horizontal — left/right axis (+Y)
        painter.drawLine(ox, 0, ox, h)  # vertical   — forward axis    (+X)

        # Axis labels — drawn in dim gray near the positive ends of each axis
        painter.setPen(QPen(DIM))
        margin = 4  # px from edge / axis line

        # +X label: top of the vertical axis (forward = up)
        painter.drawText(ox + margin, margin + 12, "+x")

        # +Y label: left edge of the horizontal axis (left = left)
        painter.drawText(margin, oy - margin, "+y")

    def _draw_path(self, painter: QPainter, w: int, h: int):
        """Draw lines between consecutive waypoints, then waypoint dots on top."""
        waypoints = [Point(0, 0)] + self.node.target_path[:]

        # Lines between consecutive waypoints
        pen = QPen(WHITE)
        pen.setWidth(2)
        painter.setPen(pen)
        painter.setBrush(Qt.BrushStyle.NoBrush)

        for a, b in zip(waypoints, waypoints[1:]):
            x1, y1 = world_to_screen(a, w, h)
            x2, y2 = world_to_screen(b, w, h)
            painter.drawLine(x1, y1, x2, y2)

        # Dots at each waypoint (drawn after lines so they sit on top)
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(PURPLE))
        r = DOT_SIZE // 2

        for pt in waypoints:
            px, py = world_to_screen(pt, w, h)
            painter.drawEllipse(px - r, py - r, DOT_SIZE, DOT_SIZE)

    def _draw_features(self, painter: QPainter, w: int, h: int):
        """Draw detected features (stop signs, traffic lights, etc.) as colored squares."""
        painter.setPen(Qt.PenStyle.NoPen)
        r = DOT_SIZE // 2

        for entity in self.node.entities:
            x, y = entity.x, entity.y
            feature_type = DEFAULT_FEATURE_TYPE
            color = FEATURE_COLORS.get(feature_type, WHITE)
            painter.setBrush(QBrush(color))

            px, py = world_to_screen(Point(x, y), w, h)
            painter.drawRect(px - r, py - r, DOT_SIZE, DOT_SIZE)

    def _draw_target(self, painter: QPainter, w: int, h: int):
        """Draw the navigation target location as a green circle."""
        if self.node.target_location is None:
            return

        tx, ty = self.node.target_location
        px, py = world_to_screen(Point(tx, ty), w, h)
        r = TARGET_SIZE // 2

        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(GREEN))
        painter.drawEllipse(px - r, py - r, TARGET_SIZE, TARGET_SIZE)

    def _draw_lanes(self, painter: QPainter, w: int, h: int):
        """Draw lane boundaries as polylines — left in purple, right in blue."""
        if self.node.lane is None:
            return

        painter.setBrush(Qt.BrushStyle.NoBrush)

        for side, color in (
            (self.node.lane.left,  PURPLE),
            (self.node.lane.right, BLUE),
        ):
            pen = QPen(color)
            pen.setWidth(2)
            painter.setPen(pen)

            for p1, p2 in zip(side, side[1:]):
                x1, y1 = world_to_screen(p1, w, h)
                x2, y2 = world_to_screen(p2, w, h)
                painter.drawLine(x1, y1, x2, y2)