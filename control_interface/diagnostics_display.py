from textual.widgets import Static

from ap1_control_interface.node import AP1SystemInterfaceNode

class DiagnosticsDisplay(Static):
    """
    All the diagnostics at the top of the display. Currently only speed, target speed
    and target location.
    """

    def __init__(self, node: AP1SystemInterfaceNode):
        super().__init__()
        self.ros_node = node 

    def on_mount(self):
        # update this screen every 0.1 sec
        self.set_interval(0.1, self.update_display)

    def update_display(self):
        speed = self.ros_node.current_speed 
        turn_angle = self.ros_node.current_turn_angle
        target_speed = self.ros_node.target_speed 
        target_loc = self.ros_node.target_location 

        # speedometer bar
        max_speed = 10.0 # m/s
        speed_percent = min(100, (speed / max_speed) * 100)
        speed_bar_len = int(speed_percent / 5) # 20 max chars
        speed_bar = "█" * speed_bar_len + "░" * (20 - speed_bar_len)

        display_text = f"""
========== VEHICLE DIAGNOSTICS ==========

- Speed:\t\t{speed:6.2f} m/s   
- Turn Angle:\t\t{turn_angle:6.2f} rads
- Target Speed:\t\t{target_speed:6.2f} m/s                                        
- Target Loc:\t\t{target_loc[0]:6.2f}, {target_loc[1]:6.2f})"""

        self.update(display_text)