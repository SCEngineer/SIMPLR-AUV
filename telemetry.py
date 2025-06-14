#!/usr/bin/env python3
"""
SIMPLR-AUV Telemetry Logger
Logs both estimated and true state information during simulation.
"""

import csv
import os
from datetime import datetime

class TelemetryLogger:
    def __init__(self, log_dir="logs"):
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(log_dir, f"telemetry_{timestamp}.csv")
        self.file = open(self.filename, "w", newline='')
        self.writer = csv.DictWriter(self.file, fieldnames=[
            "time",
            "est_x", "est_y", "est_depth", "est_heading", "est_speed",
            "true_x", "true_y", "true_depth", "true_heading", "true_speed",
            "mode", "gps_fix"
        ])
        self.writer.writeheader()

    def log(self, time_s: float, est_state: dict, true_state: dict, mode: str, gps_fix: bool):
        self.writer.writerow({
            "time": round(time_s, 2),
            "est_x": round(est_state.get("x", 0), 2),
            "est_y": round(est_state.get("y", 0), 2),
            "est_depth": round(est_state.get("depth", 0), 2),
            "est_heading": round(est_state.get("heading", 0), 2),
            "est_speed": round(est_state.get("speed", 0), 2),
            "true_x": round(true_state.get("true_x", 0), 2),
            "true_y": round(true_state.get("true_y", 0), 2),
            "true_depth": round(true_state.get("true_depth", 0), 2),
            "true_heading": round(true_state.get("true_heading", 0), 2),
            "true_speed": round(true_state.get("true_speed", 0), 2),
            "mode": mode,
            "gps_fix": gps_fix
        })

    def close(self):
        self.file.close()
        print(f"Telemetry log saved to {self.filename}")
