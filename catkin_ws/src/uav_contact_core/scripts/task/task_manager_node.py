#!/usr/bin/env python3


class TaskManager:
    """Minimal baseline task manager state holder."""

    def __init__(self):
        self.state = "IDLE"

    def on_safety_emergency(self):
        self.state = "EMERGENCY_RETREAT"
