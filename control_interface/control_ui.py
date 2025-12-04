import threading 
import subprocess
import signal
import time
import rclpy

from textual.app import App, ComposeResult
from textual.containers import Container, VerticalScroll, Horizontal
from textual.widgets import Header, Footer, Input, Label

from .node import AP1SystemInterfaceNode
from .command_output import CommandOutput
from .diagnostics_display import DiagnosticsDisplay
from .visual_path import PathCanvas

def print_help(self):
    self.command_output.add_line('=' * 40)
    self.command_output.add_line('Welcome to AP1 Debug UI')
    self.command_output.add_line('Commands:')
    self.command_output.add_line('\tspeed <value>     - Set target speed (m/s)')
    self.command_output.add_line('\tlocation <x> <y>  - Set target location (m)')
    self.command_output.add_line('\tget speed_profile - Return planned speed profile (m/s)')
    self.command_output.add_line('\tget planned_path  - Return planned path waypoints')
    self.command_output.add_line('\techo topic <topic_path> [-t s] - Echo a ROS topic')
    self.command_output.add_line('\treset             - Reset system and simulation (if applicable)')
    self.command_output.add_line('\tclear             - Clear screen')
    self.command_output.add_line('\thelp              - Print this screen')
    self.command_output.add_line('=' * 40)
        
class AP1DebugUI(App):
    # me n my homies hate writing css
    # god bless chat:
    CSS_PATH = 'style.css'

    def __init__(self, node: AP1SystemInterfaceNode):
        super().__init__()
        self.ros_node = node
        self.command_output = CommandOutput()

    def compose(self) -> ComposeResult:
        yield Header()

        with Horizontal(classes='half-height'):
            with Container(id='diagnostics_container', classes='pane'):
                yield Label("DIAGNOSTICS", classes='header')
                yield DiagnosticsDisplay(self.ros_node)
                yield Label("PLANNED PATH", classes='header')
                yield PathCanvas(self.ros_node)

            with Container(id='cli_container', classes='pane'):
                yield Label('COMMAND LINE INTERFACE', classes='header')
                with VerticalScroll(id='output_scroll'):
                    yield self.command_output 
                yield Input(placeholder='Command...', id='command_input')

        yield Footer()

    def on_mount(self):
        # start ros spin
        threading.Thread(target=rclpy.spin, args=(self.ros_node,), daemon=True).start()

        # welcome msg
        print_help(self)

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
                print_help(self)

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

            elif command == 'get':
                if len(parts) < 2:
                    self.command_output.add_line(
                        'Error! Usage: get <speed_profile | planned_path>'
                    )
                    return

                inner_command = parts[1].lower()
                if inner_command == 'speed_profile':
                    speed_profile = self.ros_node.speed_profile

                    out = ''
                    for spd in speed_profile:
                        out += spd.__str__() + ' m/s, '
                    out = out[:-2]
                    self.command_output.add_line('{ ' + out + ' }')

                elif inner_command == 'planned_path':
                    planned_path = self.ros_node.planned_path

                    out = ''
                    for waypoint in planned_path:
                        out += f'({waypoint.x:.2f}, {waypoint.y:.2f}), '
                    out = out[:-2]
                    self.command_output.add_line('{ ' + out + ' }')

                else:
                    self.command_output.add_line('Error! Unknown get command.')


            elif command == 'echo':
                if len(parts) < 3 or parts[1] != 'topic':
                    self.command_output.add_line(
                        'Error! Usage: echo topic <topic_path> [-t seconds]'
                    )
                    return

                topic_path = parts[2]
                timeout = None

                # Parse optional -t flag
                if '-t' in parts:
                    try:
                        t_index = parts.index('-t')
                        timeout = int(parts[t_index + 1])
                    except (ValueError, IndexError):
                        self.command_output.add_line(
                            'Error! -t must be followed by an integer'
                        )
                        return

                self.command_output.add_line(
                    f'Listening to topic: {topic_path}'
                )

                process = subprocess.Popen(
                    ['ros2', 'topic', 'echo', topic_path],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    text=True,
                    preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_IGN)
                )

                start_time = time.time()

                try:
                    while True:
                        # Timeout check
                        if timeout is not None:
                            if time.time() - start_time >= timeout:
                                self.command_output.add_line(
                                    f'✓ Auto-stopped after {timeout} seconds'
                                )
                                process.terminate()
                                break

                        line = process.stdout.readline()
                        if line:
                            self.command_output.add_line(line.strip())
                        else:
                            break
                except KeyboardInterrupt:
                    self.command_output.add_line('✓ Stopped by user (CTRL-C)')
                    process.terminate()

            elif command == 'reset':
                self.command_output.add_line("Not yet implemented.")

            else:
                self.command_output.add_line('Command not found.')
        except Exception as e:
            self.command_output.add_line(f"Error: {str(e)}")

