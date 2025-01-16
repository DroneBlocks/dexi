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

        thread = Thread(target=self.run, args=tuple())
        thread.start()
        # print("started timer thread!!!")

    def start(self):
        self.start_time = time.time()
        self.end_time = self.start_time + self.duration
        # print(f"Timer: Started!!! Start: {self.start_time} End: {self.end_time}")
        self.enabled = True
    
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

    def run(self):
        # print("started run func!")
        while True:
            self.time_remaining = self.get_remaining_time()
            if self.enabled:
                # print(self.time_remaining)
                if self.time_remaining == 0:
                    if self.function is not None:
                        self.enabled = False
                        # print("Timer: Calling Function!!!")
                        self.function()
            time.sleep(.01)