#!/usr/bin/env python3
"""
SIMPLR-AUV Vehicle Dynamics Model
Simulates full 6-DOF dynamics with physics-based force integration.
"""

import math
from typing import Dict, Any
import numpy as np

class VehicleDynamics:
    def __init__(self):
        # Physical properties
        self.mass = 50.0  # kg
        self.volume = 0.05  # m^3 (for buoyancy)
        self.water_density = 997.0  # kg/m^3
        self.gravity = 9.81  # m/s^2
        self.inertia = np.diag([1.0, 1.0, 2.0])  # Simplified inertia matrix [Ixx, Iyy, Izz]

        # State vectors
        self.position = np.zeros(3)     # x, y, z (ENU)
        self.velocity = np.zeros(3)     # u, v, w (body frame)
        self.orientation = np.zeros(3)  # roll, pitch, yaw
        self.ang_velocity = np.zeros(3) # p, q, r

    def simulate_motion(self, dt: float, control: Dict[str, Any]) -> Dict[str, Any]:
        # Extract control inputs
        thrust = control.get("thrust", 0.0)  # N
        rudder_angle = math.radians(control.get("rudder_deg", 0.0))
        elevator_angle = math.radians(control.get("elevator_deg", 0.0))

        # Compute net forces in body frame
        buoyant_force = self.volume * self.water_density * self.gravity
        weight_force = self.mass * self.gravity
        net_vertical_force = buoyant_force - weight_force

        drag = -0.5 * self.velocity * np.abs(self.velocity) * 5.0  # Simplified drag model
        thrust_vector = np.array([thrust, 0.0, 0.0])  # Forward only
        net_force = thrust_vector + drag + np.array([0.0, 0.0, net_vertical_force])

        # Update linear motion
        acceleration = net_force / self.mass
        self.velocity += acceleration * dt
        self.position += self._body_to_world(self.velocity, self.orientation) * dt

        # Compute torques (simplified control surface model)
        torque_yaw = rudder_angle * 2.0  # Nm
        torque_pitch = elevator_angle * 2.0  # Nm
        torque_roll = 0.0
        torque = np.array([torque_roll, torque_pitch, torque_yaw])

        ang_accel = np.linalg.inv(self.inertia) @ (torque - np.cross(self.ang_velocity, self.inertia @ self.ang_velocity))
        self.ang_velocity += ang_accel * dt
        self.orientation += self.ang_velocity * dt

        # Compute heading and derived values
        heading_deg = math.degrees(self.orientation[2]) % 360
        true_speed = np.linalg.norm(self.velocity)

        return {
            "true_lat": 34.0 + self.position[1] / 111111,  # Rough conversion y -> lat
            "true_lon": -117.0 + self.position[0] / (111111 * math.cos(math.radians(34.0))),
            "true_depth": -self.position[2],
            "true_heading": heading_deg,
            "true_speed": true_speed,
            "vx": self.velocity[0],
            "vy": self.velocity[1],
            "vz": self.velocity[2]
        }

    def _body_to_world(self, velocity: np.ndarray, orientation: np.ndarray) -> np.ndarray:
        roll, pitch, yaw = orientation
        cy, sy = math.cos(yaw), math.sin(yaw)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cr, sr = math.cos(roll), math.sin(roll)

        R = np.array([
            [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
            [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
            [-sp,   cp*sr,            cp*cr           ]
        ])
        return R @ velocity
