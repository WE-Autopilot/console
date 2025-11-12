import threading 
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32 
from geometry_msgs.msg import Point 

from textual.app import App, ComposeResult
from textual.containers import Container, VerticalScroll
from textual.widgets import Header, Footer, Input, Static, Label

class AP1DebugUINode(Node):
    def __init__(self):
        super().__init__('ap1_debug_ui')

        # pulishers
        self.speed_pub = self.create_publisher(Float32, 'ap1/control/target_speed', 10)
        self.location_pub = self.create_publisher(Point, 'ap1/control/target_location', 10)

        # subscriber
        self.speed_sub = self.create_subscription(Float32, 'ap1/acutation/speed', self.speed_callback, 10)

        self.current_speed = 0.0
        self.target_speed = 0.0
        self.target_location = (0.0, 0.0)

    def speed_callback(self, msg):
        self.current_speed = msg.data
    
    def set_target_speed(self, speed: float):
        self.target_speed = speed
        msg = Float32()
        msg.data = speed 
        self.speed_pub.publish(msg)
        self.get_logger().info(f'Sent target speed: {speed}')

    def set_target_location(self, x: float, y: float):
        self.target_location = (x, y)
        msg = Point()
        msg.x = x
        msg.y = y 
        msg.z = 0.0
        self.location_pub.publish(msg)
        self.get_logger().info(f'Sent target location: ({x}, {y})')

class DiagnosticsDisplay(Static):
    """
    All the diagnostics at the top of the display. Currently only speed, target speed
    and target location.
    """

    def __init__(self, node: AP1DebugUINode):
        super().__init__()
        self.ros_node = node 

    def on_mount(self):
        # update this screen every 0.1 sec
        self.set_interval(0.1, self.update_display)

    def update_display(self):
        speed = self.ros_node.current_speed 
        target_speed = self.ros_node.target_speed 
        target_loc = self.ros_node.target_location 

        # speedometer bar
        max_speed = 10.0 # m/s
        speed_percent = min(100, (speed / max_speed) * 100)
        speed_bar_len = int(speed_percent / 5) # 20 max chars
        speed_bar = "█" * speed_bar_len + "░" * (20 - speed_bar_len)

        display_text = f"""
========== VEHICLE DIAGNOSTICS ==========

- Speed:        {speed:6.2f} m/s   
- Target Speed: {target_speed:6.2f} m/s                                        
- Target Loc:   ({target_loc[0]:6.2f}, {target_loc[1]:6.2f})"""

        self.update(display_text)

class CommandOutput(Static):
    """Display for cmd history + outputs"""

    def __init__(self):
        super().__init__()
        self.history = deque(maxlen=100)

    def add_line(self, text: str):
        self.history.append(text)
        self.update('\n'.join(self.history))

class AP1DebugUI(App):
    # me n my homies hate writing css
    # god bless chat:
    CSS = """
    Screen {
        layout: vertical;
    }
    
    DiagnosticsDisplay {
        height: 10;
        background: $surface;
        border: solid $primary;
        margin: 1;
        padding: 1;
    }
    
    #cli_container {
        height: 1fr;
        border: solid $accent;
        margin: 1;
    }
    
    #cli_header {
        background: $accent;
        color: $text;
        height: 1;
        content-align: center middle;
        text-style: bold;
    }
    
    #output_scroll {
        height: 1fr;
        background: $surface;
        padding: 1;
    }
    
    CommandOutput {
        height: auto;
    }
    
    #input_container {
        height: 5;
        background: $panel;
        padding: 1;
    }
    
    Input {
        width: 100%;
    }
    """

    def __init__(self, node: AP1DebugUINode):
        super().__init__()
        self.ros_node = node
        self.command_output = CommandOutput()

    def compose(self) -> ComposeResult:
        yield Header()
        yield DiagnosticsDisplay(self.ros_node)

        with Container(id='cli_container'):
            yield Label('COMMAND LINE INTERFACE', id='cli_header')
            with VerticalScroll(id='output_scroll'):
                yield self.command_output 
            with Container(id='input_container'):
                yield Input(placeholder='>', id='command_input')

        yield Footer()

    def on_mount(self):
        # start ros spin
        threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True).start()

        # welcome msg
        self.command_output.add_line('=' * 20)
        self.command_output.add_line('Welcome to AP1 Debug UI')
        self.command_output.add_line('Commands:')
        self.command_output.add_line('\tspeed <value>\t\t- Set target speed (m/s)')
        self.command_output.add_line('\tlocation <x> <y>\t\t- Set target location (m)')
        self.command_output.add_line('\treset\t\t- Reset system and simulation (if applicable)')
        self.command_output.add_line('\tclear\t\t- Clear screen')
        self.command_output.add_line('\thelp\t\t- Print this screen')
        self.command_output.add_line('=' * 20)

        # focus the input
        self.query_one('#command_input', Input).focus()

    def on_input_submitted(self, event: Input.Submitted):
        cmd = event.value.strip()

        # skip if nothing
        if not cmd:
            return 

        # show command in output
        self.command_output.add_line(f'> {cmd}')

        # clear input
        event.input.value = ''

        # execute command
        self.execute_command(cmd)

    def execute_command(self, cmd: str):
        try:
            parts = cmd.split() 
            command = parts[0].lower()

            if command == 'help':
                self.command_output.add_line('Commands:')
                self.command_output.add_line('\tspeed <value>\t\t- Set target speed (m/s)')
                self.command_output.add_line('\tlocation <x> <y>\t\t- Set target location (m)')
                self.command_output.add_line('\treset\t\t- Reset system and simulation (if applicable)')
                self.command_output.add_line('\tclear\t\t- Clear screen')
                self.command_output.add_line('\thelp\t\t- Print this screen')
                self.command_output.add_line('=' * 20)

            elif command == 'clear':
                self.command_output.history.clear()
                self.command_output.update('')
                self.command_output.add_line('History cleared') # may remove if annoying

            elif command == 'speed':
                if len(parts) < 2:
                    self.command_output.add_line('Error! Usage: speed <value>')
                else:
                    speed = float(parts[1]) # grab the value
                    self.ros_node.set_target_speed(speed)
                    self.command_output.add_line(f"✓ Target speed set to {speed:.2f} m/s")

            elif command == 'location':
                if len(parts) < 3:
                    self.command_output.add_line('Error! Usage: location <x> <y>')
                else:
                    x, y = float(parts[1]), float(parts[2])
                    self.ros_node.set_target_location(x, y)
                    self.command_output.add_line(f"✓ Target location set to ({x}, {y})")

            elif command == 'reset':
                self.command_output.add_line("Not yet implemented.")

            else:
                self.command_output.add_line('Command not found.')
        except Exception as e:
            self.command_output.add_line(f"Error: {str(e)}")

# main time bbgl
def main(args=None):
    rclpy.init(args=args)
    ros_node = AP1DebugUINode()
    app = AP1DebugUI(ros_node)

    try:
        # run
        app.run()
    finally:
        # teardown
        ros_node.destroy_node()
        rclpy.shutdown()

# bismAllah
if __name__ == '__main__':
    main()
