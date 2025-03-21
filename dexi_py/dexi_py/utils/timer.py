from threading import Thread
from typing import Any
import time

class Timer(object):
    def __init__(self):
        self.init_duration = 0
        self.duration = 0
        self.enabled = False
        self.function: Any = None
        self.start_time = 0
        self.end_time = 0
        self.thread = None  # Don't start the thread in the constructor

    def start(self):
        self.start_time = time.time()
        self.end_time = self.start_time + self.duration
        self.enabled = True
        if self.thread is None or not self.thread.is_alive():  # Start the thread if not already running
            self.thread = Thread(target=self.run)
            self.thread.start()

    def pause(self):
        self.enabled = False
        self.duration = self.get_remaining_time()

    def reset(self):
        self.enabled = False
        self.duration = self.init_duration
        self.start_time = 0
        self.end_time = 0

    def set_timeout(self, duration):
        self.enabled = False
        self.init_duration = duration
        self.duration = duration

    def get_remaining_time(self):
        remaining = self.end_time - time.time()
        return remaining if remaining > 0 else 0

    def stop(self):
        """Stops the timer and exits the thread."""
        self.enabled = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1)  # Wait for thread to terminate gracefully

    def run(self):
        while self.enabled:
            self.time_remaining = self.get_remaining_time()
            if self.enabled and self.time_remaining == 0:
                if self.function is not None:
                    self.enabled = False
                    self.function()
            time.sleep(0.01)  # Sleep to avoid high CPU usage
