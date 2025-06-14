#!/usr/bin/env python3
"""
SIMPLR-AUV Control System
PID controllers with proper indentation - Windows 11 compatible.
"""

import asyncio
import logging
import math
import time
from typing import Dict, Any, Optional
from dataclasses import dataclass
from collections import deque

from core_types import ControlMode, VehicleCommand

@dataclass
class ControlConfig:
    """Control system configuration parameters"""
    control_rate: float = 50.0
    depth_kp: float = 1.0
    depth_ki: float = 0.05
    depth_kd: float = 0.3
    attitude_kp: float = 3.0
    attitude_ki: float = 0.2
    attitude_kd: float = 0.8
    max_thrust: float = 1.0
    max_depth: float = 20.0

class PIDController:
    """Simple PID controller"""
    
    def __init__(self, kp: float, ki: float, kd: float, output_limit: float = 1.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = None

    def update(self, setpoint: float, measurement: float, dt: float = 0.1) -> float:
        """Update PID controller"""
        error = setpoint - measurement
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term
        self.integral_error += error * dt
        self.integral_error = max(-10.0, min(10.0, self.integral_error))
        i_term = self.ki * self.integral_error
        
        # Derivative term
        if self.last_error is not None:
            d_term = self.kd * (error - self.last_error) / dt
        else:
            d_term = 0.0
        
        # Calculate output
        output = p_term + i_term + d_term
        output = max(-self.output_limit, min(self.output_limit, output))
        
        self.last_error = error
        return output

    def reset(self):
        """Reset PID controller"""
        self.integral_error = 0.0
        self.last_error = 0.0

    def set_gains(self, kp: float, ki: float, kd: float):
        """Update PID gains"""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def get_error_stats(self) -> Dict[str, float]:
        """Get error statistics"""
        return {
            'last_error': self.last_error or 0.0,
            'integral': self.integral_error,
            'kp': self.kp,
            'ki': self.ki,
            'kd': self.kd
        }

class ControlSystem:
    """Main control system"""
    
    def __init__(self, config: ControlConfig):
        self.config = config
        self.is_active = False
        self.control_mode = ControlMode.AUTOMATIC
        
        # Initialize PID controllers
        self.depth_controller = PIDController(
            config.depth_kp, config.depth_ki, config.depth_kd, config.max_thrust
        )
        self.heading_controller = PIDController(
            config.attitude_kp, config.attitude_ki, config.attitude_kd, 1.0
        )
        self.speed_controller = PIDController(
            1.0, 0.1, 0.2, config.max_thrust
        )
        
        # Control targets
        self.target_depth = 0.0
        self.target_heading = 0.0
        self.target_speed = 0.0
        
        # Emergency states
        self.emergency_surface_active = False
        self.emergency_stop_active = False
        
        self.logger = logging.getLogger("ControlSystem")

    async def start(self):
        """Start control system"""
        self.is_active = True
        self.logger.info("Control system started")

    async def stop(self):
        """Stop control system"""
        self.is_active = False
        self.logger.info("Control system stopped")

    def update(self, guidance_commands: Dict[str, Any], 
              nav_state: Dict[str, Any], dt: float) -> Dict[str, Any]:
        """Main control update function"""
        
        if not self.is_active:
            return self._get_safe_commands()
        
        if self.emergency_stop_active:
            return self._get_emergency_stop_commands()
        
        if self.emergency_surface_active:
            return self._get_emergency_surface_commands(nav_state, dt)
        
        # Normal control
        return self._automatic_control(guidance_commands, nav_state, dt)

    def _automatic_control(self, guidance_commands: Dict[str, Any], 
                          nav_state: Dict[str, Any], dt: float) -> Dict[str, Any]:
        """Execute automatic control"""
        
        # Extract current state
        current_position = nav_state.get('position', {})
        current_attitude = nav_state.get('attitude', {})
        current_velocity = nav_state.get('velocity', {})
        
        current_depth = current_position.get('depth', 0.0)
        current_heading = current_attitude.get('yaw', 0.0)
        current_speed = current_velocity.get('magnitude', 0.0)
        
        # Extract targets from guidance
        self.target_depth = guidance_commands.get('depth_m', current_depth)
        self.target_heading = guidance_commands.get('heading_deg', current_heading)
        self.target_speed = guidance_commands.get('speed_mps', 0.0)
        
        # Calculate control outputs
        depth_output = self.depth_controller.update(self.target_depth, current_depth, dt)
        
        # Handle heading wraparound
        heading_error = self.target_heading - current_heading
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
        
        yaw_output = self.heading_controller.update(0.0, -heading_error, dt)
        surge_output = self.speed_controller.update(self.target_speed, current_speed, dt)
        
        return {
            'surge': surge_output,
            'sway': 0.0,
            'heave': depth_output,
            'yaw_rate': yaw_output,
            'control_mode': self.control_mode.value
        }

    def _get_emergency_surface_commands(self, nav_state: Dict[str, Any], dt: float) -> Dict[str, Any]:
        """Emergency surface commands"""
        current_depth = nav_state.get('position', {}).get('depth', 0.0)
        
        if current_depth <= 1.0:
            self.emergency_surface_active = False
        
        return {
            'surge': 0.0,
            'sway': 0.0,
            'heave': -1.0,  # Maximum ascent
            'yaw_rate': 0.0,
            'control_mode': 'EMERGENCY'
        }

    def _get_emergency_stop_commands(self) -> Dict[str, Any]:
        """Emergency stop commands"""
        return {
            'surge': 0.0,
            'sway': 0.0,
            'heave': 0.0,
            'yaw_rate': 0.0,
            'control_mode': 'EMERGENCY'
        }

    def _get_safe_commands(self) -> Dict[str, Any]:
        """Safe neutral commands"""
        return {
            'surge': 0.0,
            'sway': 0.0,
            'heave': 0.0,
            'yaw_rate': 0.0,
            'control_mode': 'SAFE'
        }

    def emergency_surface(self):
        """Activate emergency surface procedure"""
        self.emergency_surface_active = True
        self.logger.critical("Emergency surface activated")

    def emergency_stop(self):
        """Activate emergency stop"""
        self.emergency_stop_active = True
        self.logger.critical("Emergency stop activated")

    def reset_emergency(self):
        """Reset emergency states"""
        self.emergency_surface_active = False
        self.emergency_stop_active = False

    def reset_all_controllers(self):
        """Reset all PID controllers"""
        self.depth_controller.reset()
        self.heading_controller.reset()
        self.speed_controller.reset()

    def set_control_mode(self, mode):
        """Set control mode"""
        self.control_mode = mode

    def get_control_status(self) -> Dict[str, Any]:
        """Get control system status"""
        return {
            'active': self.is_active,
            'mode': self.control_mode.value if hasattr(self.control_mode, 'value') else str(self.control_mode),
            'emergency_surface': self.emergency_surface_active,
            'emergency_stop': self.emergency_stop_active,
            'targets': {
                'depth': self.target_depth,
                'heading': self.target_heading,
                'speed': self.target_speed
            },
            'controller_errors': {
                'depth': self.depth_controller.get_error_stats(),
                'heading': self.heading_controller.get_error_stats(),
                'speed': self.speed_controller.get_error_stats()
            }
        }

def create_default_control_config() -> ControlConfig:
    """Create default control configuration"""
    return ControlConfig()
