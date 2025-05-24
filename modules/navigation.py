# modules/navigation.py

import math

class Navigation:
    def __init__(self):
        self.state = {
            "position": [0.0, 0.0],  # x, y in meters
            "heading": 0.0,         # degrees
            "depth": 0.0,           # meters
            "speed": 0.0            # meters per second
        }
        print("[NAVIGATION] Initialized")

    def get_state(self):
        return self.state

    def update(self, dt=1.0):
        """
        Update the position of the AUV based on current speed and heading.
        dt: timestep in seconds
        """
        heading_rad = math.radians(self.state["heading"])
        dx = self.state["speed"] * math.cos(heading_rad) * dt
        dy = self.state["speed"] * math.sin(heading_rad) * dt

        self.state["position"][0] += dx
        self.state["position"][1] += dy

        print(f"[NAVIGATION] Updated position to {self.state['position']}, depth {self.state['depth']:.1f}m")

    def set_heading(self, heading):
        self.state["heading"] = heading % 360
        print(f"[NAVIGATION] Heading set to {self.state['heading']:.1f} degrees")

    def set_speed(self, speed):
        self.state["speed"] = speed
        print(f"[NAVIGATION] Speed set to {self.state['speed']:.2f} m/s")

    def set_depth(self, depth):
        self.state["depth"] = depth
        print(f"[NAVIGATION] Depth set to {self.state['depth']:.2f} meters")
