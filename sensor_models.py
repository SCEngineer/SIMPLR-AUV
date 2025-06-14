#!/usr/bin/env python3
"""
SIMPLR-AUV Sensor Suite Simulation
Provides simulated sensor readings based on vehicle state.
"""

import random
import math

class SensorSuite:
    def __init__(self, vehicle):
        self.vehicle = vehicle

    def read_all(self, time_s: float) -> dict:
        state = self.vehicle

        # Get true values from dynamics
        true_state = state.simulate_motion(0, {})  # No update, just get current

        # Simulate noisy sensors
        return {
            "depth": true_state["true_depth"] + random.gauss(0, 0.1),
            "compass": true_state["true_heading"] + random.gauss(0, 1.0),
            "gyro": random.gauss(0, 0.01),  # Simulated heading rate (optional real model)
            "water_speed_x": true_state["vx"] + random.gauss(0, 0.1),
            "gps_fix": (time_s % 30) < 5,  # GPS fix available every 30s for 5s
            "gps_x": true_state.get("true_x", 0.0) + random.gauss(0, 2.0),
            "gps_y": true_state.get("true_y", 0.0) + random.gauss(0, 2.0)
        }