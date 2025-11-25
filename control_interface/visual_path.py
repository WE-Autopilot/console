"""
Visualizes Car path
Notice that +x is FORWARD and +y is LEFT relative to the CAR.
"""
from textual_canvas import Canvas 
from textual.color import Color

from node import AP1SystemInterfaceNode

WHITE = Color.parse("white")
DIM = Color.parse("darkgrey")
RED = Color.parse("red")
GREEN = Color.parse("green")
YELLOW = Color.parse("yellow")
PURPLE = Color.parse("purple")
BLUE = Color.parse("blue")

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

    # scale down
    x = int(point.x / MAX_X * DEFAULT_WIDTH)
    y = int(point.y / MAX_Y * DEFAULT_HEIGHT)

    # rotate
    # +x on the car is +y on canvas, +y on the car is +x on the canvas
    return y, x


class PathCanvas(Canvas):
    refresh_rate = 10 # Hz

    def __init__(self, node: AP1SystemInterfaceNode, **kwargs):
        # width and height is PIXELS not SIZE!
        super().__init__(width=DEFAULT_WIDTH, height=DEFAULT_HEIGHT, **kwargs)
        self.node = node

    def on_mount(self):
        # update at 10hz
        self.set_interval(1 / self.refresh_rate, self.draw_path)

    def draw_path(self):
        waypoints = self.node.target_path # these are in meters (global coords)
        waypoints.insert(0, Point(0, 0)) # append origin
        width, height = self.width, self.height 

        # (0,0) is bottom center
        cx = width // 2
        cy = height - 1

        self.clear()

        # draw axes 
        for x in range(width):
            self.set_pixel(x, cy, DIM)
        for y in range(height):
            self.set_pixel(cx, y, DIM)
        
        # plot waypoints & connect with lines
        for i, point in enumerate(waypoints):
            x, y = global_to_canvas_coords(point)
            sx = cx + x # shift origin to center
            sy = cy - y # flip y upward

            # connect to next waypoint
            if i < len(waypoints) - 1:
                next_point = waypoints[i + 1]

                nx, ny = global_to_canvas_coords(next_point)
                nx, ny = cx + nx, cy - ny
                self.draw_line(sx, sy, nx, ny, WHITE)

        # draw waypoints on top
        for i, point in enumerate(waypoints):
            x, y = global_to_canvas_coords(point)
            sx = cx + x # shift origin to center
            sy = cy - y # flip y upward
            self.set_pixel(sx, sy, PURPLE)

        # draw features on top
        for feature in self.node.features:
            feature_type, x, y = feature

            # convert global coords to canvas
            sx, sy = global_to_canvas_coords(Point(x, y))
            canvas_cx = self.width // 2
            canvas_cy = self.height - 1
            sx = canvas_cx + sx 
            sy = canvas_cy - sy

            # color mapping for features
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

            if 0 <= sx < self.width and 0 <= sy < self.height:
                self.set_pixel(sx, sy, color)


 