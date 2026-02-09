"""
Visualizes Car path
Notice that +x is FORWARD and +y is LEFT relative to the CAR.
"""
from PyQt6.QtWidgets import QWidget
from PyQt6.QtGui import QPainter, QPen, QBrush
from PyQt6.QtCore import Qt, QTimer

from .node import AP1SystemInterfaceNode

WHITE = Qt.GlobalColor.white
DIM = Qt.GlobalColor.darkGray
RED = Qt.GlobalColor.red
GREEN = Qt.GlobalColor.green
YELLOW = Qt.GlobalColor.yellow
PURPLE = Qt.GlobalColor.darkMagenta
BLUE = Qt.GlobalColor.blue
BLACK = Qt.GlobalColor.black

# EXAMPLE WAYPOINTS
DEBUG_WAYPOINTS = [(0,0), (5,15), (10,20), (20,25)]

# COORDS
DEFAULT_WIDTH = 60
DEFAULT_HEIGHT = 60

class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y

def global_to_canvas_coords(point: Point) -> tuple[int, int]:
    MAX_X, MAX_Y = 20, 20 # m

    """Helper to get x,y safely from either an object or a tuple."""
    if hasattr(point, 'x') and hasattr(point, 'y'):
        px, py = point.x, point.y
    elif isinstance(point, (tuple, list)) and len(point) >= 2:
        px, py = point[0], point[1]
    else:
        px, py = 0, 0

    # scale down
    x = int(px / MAX_X * DEFAULT_WIDTH)
    y = int(py / MAX_Y * DEFAULT_HEIGHT)

    # rotate
    # +x on the car is +y on canvas, +y on the car is +x on the canvas
    return y, x


class PathCanvas(QWidget):
    refresh_rate = 10 # Hz

    def __init__(self, node: AP1SystemInterfaceNode, parent=None):
        # width and height is PIXELS not SIZE!
        super().__init__(parent)
        self.node = node

        self.setMinimumSize(300, 300)

        # Style sheet
        self.setStyleSheet("background: black;")

        # Update display
        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(int(1000 / self.refresh_rate))

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)

        waypoints = self.node.target_path[:] # these are in meters (global coords)
        waypoints.insert(0, Point(0, 0)) # append origin
        width, height = self.width(), self.height()

        painter.fillRect(0, 0, width, height, BLACK)

        scale_x = width / DEFAULT_WIDTH
        scale_y = height / DEFAULT_HEIGHT

        def logical_to_screen(lx, ly):
            return lx * scale_x, ly * scale_y

        # (0,0) is bottom center
        cx = DEFAULT_WIDTH // 2
        cy = DEFAULT_HEIGHT - 1

        # draw axes
        pen_dim = QPen(DIM)
        pen_dim.setWidth(1)
        painter.setPen(pen_dim)
        
        x_screen, y_screen = logical_to_screen(cx, cy)

        painter.drawLine(0, int(y_screen), width, int(y_screen))
        painter.drawLine(int(x_screen), 0, int(x_screen), height)
        
        # plot waypoints & connect with lines
        pen_white = QPen(WHITE)
        pen_white.setWidth(2)
        painter.setPen(pen_white)

        for i, point in enumerate(waypoints):
            if i < len(waypoints) - 1:
                # Current point
                x, y = global_to_canvas_coords(point)
                sx = cx + x # shift origin to center
                sy = cy - y # flip y upward

                next_point = waypoints[i + 1]
                nx, ny = global_to_canvas_coords(next_point)
                nx, ny = cx + nx, cy - ny
                
                px1, py1 = logical_to_screen(sx, sy)
                px2, py2 = logical_to_screen(nx, ny)

                painter.drawLine(int(px1), int(py1), int(px2), int(py2))

        # draw waypoints on top
        painter.setPen(Qt.PenStyle.NoPen)
        painter.setBrush(QBrush(PURPLE))

        dot_size = 6

        for i, point in enumerate(waypoints):
            x, y = global_to_canvas_coords(point)
            sx = cx + x # shift origin to center
            sy = cy - y # flip y upward
            
            px, py = logical_to_screen(sx, sy)
            
            painter.drawEllipse(int(px - dot_size / 2), int(py - dot_size / 2), dot_size, dot_size)

        # draw features on top
        for feature in self.node.features:
            feature_type, x, y = feature

            # convert global coords to canvas
            sx, sy = global_to_canvas_coords(Point(x, y))
            sx = cx + sx
            sy = cy - sy

            px, py = logical_to_screen(sx, sy)

            # map colors to features
            if feature_type == 'stop_sign':
                color = RED
            elif feature_type == 'traffic_light':
                color = GREEN
            elif feature_type == 'stop_line':
                color = YELLOW
            elif feature_type == 'yield_sign':
                color = BLUE
            else:
                color = WHITE

            painter.setBrush(QBrush(color))
            painter.drawRect(int(px - dot_size / 2), int(py - dot_size / 2), dot_size, dot_size)

