from textual.widgets import Static, Label
from textual.containers import VerticalScroll, Container
from .node import AP1SystemInterfaceNode

class DiagnosticsDisplay(Container):
    """
    All the diagnostics at the top of the display.
    """

    def __init__(self, node: AP1SystemInterfaceNode):
        super().__init__()
        self.ros_node = node

    def compose(self):
        with VerticalScroll(id="diagnostics_scroll"):
            yield Static("", id="diagnostics_content")

    def on_mount(self):
        self.content = self.query_one("#diagnostics_content", Static)
        self.set_interval(0.1, self.update_display)

    def update_display(self):
        speed = self.ros_node.current_speed
        turn_angle = self.ros_node.current_turn_angle
        target_speed = self.ros_node.target_speed
        target_loc = self.ros_node.target_location
        motor_power = self.ros_node.motor_power * 100

        display_text = f"""Actuation:
- Speed:\t{speed:6.2f} m/s
- Turn Angle:\t{turn_angle:6.2f} rads

Control Interface:
- Target Speed:\t{target_speed:6.2f} m/s
- Target Loc:\t{target_loc[0]:6.2f}, {target_loc[1]:6.2f}

Control:
- Motor power:\t{motor_power:6.2f} %
"""
        self.content.update(display_text)
