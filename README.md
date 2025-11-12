# AP1 Control Interface

This allows users to communicate with the car.

Make sure to set up a venv with the exact versions of textual and rclpy and everything
in the requirements.txt instead of installing the packages through apt.

This is to avoid apt screwing you with a version of textual from 1200 BC.

## Running

You'll have to launch this through python by doing `python control_interface/main.py` instead of
a ros2 command. I haven't yet been able to figure out how to get ros2 to see pkgs installed in venv or
use the interpreter/pip located in .venv.


