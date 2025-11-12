from collections import deque

from textual.widgets import Static

class CommandOutput(Static):
    """Display for cmd history + outputs"""

    def __init__(self):
        super().__init__()
        self.history = deque(maxlen=100)

    def add_line(self, text: str):
        self.history.append(text)
        self.update('\n'.join(self.history))