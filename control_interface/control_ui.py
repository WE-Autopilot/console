import threading 

import rclpy

from textual.app import App, ComposeResult
from textual.containers import Container, VerticalScroll
from textual.widgets import Header, Footer, Input, Label

from ap1_control_interface.node import AP1SystemInterfaceNode
from ap1_control_interface.command_output import CommandOutput
from ap1_control_interface.diagnostics_display import DiagnosticsDisplay

class AP1DebugUI(App):
    # me n my homies hate writing css
    # god bless chat:
    CSS_PATH = "styles.css"

    def __init__(self, node: AP1SystemInterfaceNode):
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

