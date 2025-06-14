#!/usr/bin/env python3
"""
SIMPLR-AUV Actuator Module
Handles X-tail control surface mixing and thruster PWM output generation.
Corrected for proper X-tail mixing and hardware abstraction compatibility.
"""

import logging
import math
from typing import Dict, Any

class ActuatorModule:
    """
    X-tail actuator control system with proper control mixing algorithms.
    Converts high-level control commands to individual actuator positions.
    """
    
    def __init__(self,
                 xtail_max_angle: float = 30.0,
                 neutral_pwm: int = 1500,
                 pwm_range: int = 400):
        """
        Initialize actuator module with X-tail configuration.
        
        Args:
            xtail_max_angle: Maximum control surface angle in degrees (±30°)
            neutral_pwm: Neutral thruster PWM value (1500 µs)
            pwm_range: PWM range from neutral to full scale (±400 µs)
        """
        self.xtail_max_angle = xtail_max_angle
        self.neutral_pwm = neutral_pwm
        self.pwm_range = pwm_range
        self.logger = logging.getLogger("ActuatorModule")

        # Current actuator outputs
        self.port_upper_angle = 0.0
        self.starboard_upper_angle = 0.0
        self.port_lower_angle = 0.0
        self.starboard_lower_angle = 0.0
        self.thrust_pwm = neutral_pwm
        
        # Actuator health monitoring
        self.actuator_health = {
            'port_upper': True,
            'starboard_upper': True,
            'port_lower': True,
            'starboard_lower': True,
            'thruster': True
        }
        
        # Rate limiting for smooth actuator movement
        self.max_rate_deg_per_sec = 60.0  # Maximum servo rate
        self.last_update_time = None
        self.previous_angles = {
            'port_upper': 0.0,
            'starboard_upper': 0.0,
            'port_lower': 0.0,
            'starboard_lower': 0.0
        }
        
        self.logger.info(f"Actuator module initialized: ±{xtail_max_angle}° X-tail, PWM {neutral_pwm}±{pwm_range}")

    def update(self, control_commands: Dict[str, Any], dt: float) -> Dict[str, Any]:
        """
        Process control commands and apply X-tail mixing with rate limiting.
        
        Args:
            control_commands: Dictionary with control inputs
            dt: Time delta for rate limiting
            
        Returns:
            Dictionary with actuator commands ready for hardware interface
        """
        import time
        current_time = time.time()
        
        # Extract control inputs with proper defaults
        rudder = self._extract_and_clamp(control_commands, "rudder", -1.0, 1.0)
        elevator = self._extract_and_clamp(control_commands, "elevator", -1.0, 1.0)
        thrust = self._extract_and_clamp(control_commands, "thrust", -1.0, 1.0)
        
        # Alternative input names for compatibility
        if rudder == 0.0:
            rudder = self._extract_and_clamp(control_commands, "yaw_rate", -1.0, 1.0)
        if elevator == 0.0:
            elevator = self._extract_and_clamp(control_commands, "heave", -1.0, 1.0)
        if thrust == 0.0:
            thrust = self._extract_and_clamp(control_commands, "surge", -1.0, 1.0)

        # Apply X-tail control mixing
        target_angles = self._calculate_xtail_mixing(elevator, rudder)
        
        # Apply rate limiting if dt is available
        if dt > 0 and self.last_update_time is not None:
            actual_dt = min(dt, 0.1)  # Cap dt to prevent large jumps
            target_angles = self._apply_rate_limiting(target_angles, actual_dt)
        
        # Update current positions
        self.port_upper_angle = target_angles['port_upper']
        self.starboard_upper_angle = target_angles['starboard_upper']
        self.port_lower_angle = target_angles['port_lower']
        self.starboard_lower_angle = target_angles['starboard_lower']
        
        # Update thruster PWM
        self.thrust_pwm = self._calculate_thruster_pwm(thrust)
        
        # Store for next iteration
        self.previous_angles = target_angles.copy()
        self.last_update_time = current_time
        
        return self.get_actuator_commands()

    def _extract_and_clamp(self, commands: Dict[str, Any], key: str, 
                          min_val: float, max_val: float) -> float:
        """Extract and clamp command value"""
        try:
            value = float(commands.get(key, 0.0))
            return max(min_val, min(max_val, value))
        except (TypeError, ValueError):
            self.logger.warning(f"Invalid command value for {key}: {commands.get(key)}")
            return 0.0

    def _calculate_xtail_mixing(self, elevator: float, rudder: float) -> Dict[str, float]:
        """
        Calculate X-tail control surface angles using proper mixing equations.
        
        X-tail mixing (from technical description):
        - Port Upper = +elevator + rudder
        - Starboard Upper = +elevator - rudder  
        - Port Lower = -elevator + rudder
        - Starboard Lower = -elevator - rudder
        
        Args:
            elevator: Pitch command (-1.0 to +1.0)
            rudder: Yaw command (-1.0 to +1.0)
            
        Returns:
            Dictionary with target angles for each surface
        """
        # Convert normalized commands to angles
        elevator_angle = elevator * self.xtail_max_angle
        rudder_angle = rudder * self.xtail_max_angle
        
        # X-tail mixing equations
        angles = {
            'port_upper': elevator_angle + rudder_angle,
            'starboard_upper': elevator_angle - rudder_angle,
            'port_lower': -elevator_angle + rudder_angle,
            'starboard_lower': -elevator_angle - rudder_angle
        }
        
        # Apply angle limits to each surface
        for surface in angles:
            angles[surface] = self._clamp_angle(angles[surface])
            
            # Check for actuator health
            if not self.actuator_health.get(surface, True):
                angles[surface] = 0.0  # Safe position for failed actuator
        
        return angles

    def _apply_rate_limiting(self, target_angles: Dict[str, float], dt: float) -> Dict[str, float]:
        """Apply rate limiting to prevent excessive actuator speeds"""
        limited_angles = {}
        max_change = self.max_rate_deg_per_sec * dt
        
        for surface, target in target_angles.items():
            previous = self.previous_angles.get(surface, 0.0)
            change = target - previous
            
            # Limit the rate of change
            if abs(change) > max_change:
                limited_change = math.copysign(max_change, change)
                limited_angles[surface] = previous + limited_change
            else:
                limited_angles[surface] = target
        
        return limited_angles

    def _calculate_thruster_pwm(self, thrust: float) -> int:
        """
        Calculate thruster PWM from normalized thrust command.
        
        Args:
            thrust: Normalized thrust (-1.0 to +1.0)
            
        Returns:
            PWM value in microseconds (1100-1900)
        """
        if not self.actuator_health.get('thruster', True):
            return self.neutral_pwm  # Safe neutral for failed thruster
        
        pwm = int(self.neutral_pwm + thrust * self.pwm_range)
        return max(1100, min(1900, pwm))  # Hardware PWM limits

    def _clamp_angle(self, angle: float) -> float:
        """Clamp control surface angle to physical limits"""
        return max(-self.xtail_max_angle, min(self.xtail_max_angle, angle))

    def get_actuator_commands(self) -> Dict[str, Any]:
        """
        Return current actuator commands in format compatible with hardware abstraction layer.
        Windows 11 compatible - uses only standard library functions.
        
        Returns:
            Dictionary with actuator commands ready for send_actuator_commands()
        """
        return {
            "port_upper_angle": round(self.port_upper_angle, 2),
            "starboard_upper_angle": round(self.starboard_upper_angle, 2),
            "port_lower_angle": round(self.port_lower_angle, 2),
            "starboard_lower_angle": round(self.starboard_lower_angle, 2),
            "thrust_pwm": self.thrust_pwm
        }

    def get_actuator_status(self) -> Dict[str, Any]:
        """Return detailed actuator system status"""
        return {
            "angles": {
                "port_upper": self.port_upper_angle,
                "starboard_upper": self.starboard_upper_angle,
                "port_lower": self.port_lower_angle,
                "starboard_lower": self.starboard_lower_angle
            },
            "thrust_pwm": self.thrust_pwm,
            "health": self.actuator_health.copy(),
            "limits": {
                "max_angle": self.xtail_max_angle,
                "max_rate": self.max_rate_deg_per_sec,
                "pwm_range": f"{self.neutral_pwm - self.pwm_range}-{self.neutral_pwm + self.pwm_range}"
            }
        }

    def set_actuator_health(self, actuator: str, healthy: bool):
        """Set health status for specific actuator (for fault simulation)"""
        if actuator in self.actuator_health:
            self.actuator_health[actuator] = healthy
            self.logger.warning(f"Actuator {actuator} health set to {healthy}")
        else:
            self.logger.error(f"Unknown actuator: {actuator}")

    def emergency_center_surfaces(self):
        """Center all control surfaces for emergency conditions"""
        self.port_upper_angle = 0.0
        self.starboard_upper_angle = 0.0
        self.port_lower_angle = 0.0
        self.starboard_lower_angle = 0.0
        self.thrust_pwm = self.neutral_pwm
        self.logger.warning("Emergency: All control surfaces centered")

    def test_actuator_range(self) -> Dict[str, bool]:
        """
        Test full range of motion for all actuators (for system checkout).
        Windows 11 compatible version.
        
        Returns:
            Dictionary indicating test success for each actuator
        """
        test_results = {}
        
        try:
            # Test control surfaces through full range
            test_angles = [-self.xtail_max_angle, 0.0, self.xtail_max_angle]
            
            for surface in ['port_upper', 'starboard_upper', 'port_lower', 'starboard_lower']:
                try:
                    for angle in test_angles:
                        # Simulate setting angle (in real hardware, would move servo)
                        clamped_angle = self._clamp_angle(angle)
                        if abs(clamped_angle - angle) > 0.1:
                            test_results[surface] = False
                            break
                    else:
                        test_results[surface] = True
                except Exception as e:
                    self.logger.error(f"Actuator test failed for {surface}: {e}")
                    test_results[surface] = False
            
            # Test thruster PWM range
            try:
                test_pwms = [1100, 1500, 1900]
                for pwm in test_pwms:
                    # Simulate setting PWM (in real hardware, would command ESC)
                    clamped_pwm = max(1100, min(1900, pwm))
                    if clamped_pwm != pwm:
                        test_results['thruster'] = False
                        break
                else:
                    test_results['thruster'] = True
            except Exception as e:
                self.logger.error(f"Thruster test failed: {e}")
                test_results['thruster'] = False
                
        except Exception as e:
            self.logger.error(f"Actuator range test failed: {e}")
            # Return failed status for all actuators
            test_results = {name: False for name in self.actuator_health.keys()}
        
        return test_results

    def calculate_control_effectiveness(self, current_angles: Dict[str, float]) -> Dict[str, float]:
        """
        Calculate control effectiveness based on current surface positions.
        Useful for adaptive control algorithms.
        
        Args:
            current_angles: Current control surface angles
            
        Returns:
            Dictionary with effectiveness factors (0.0 to 1.0)
        """
        effectiveness = {}
        
        for surface, angle in current_angles.items():
            if surface in self.actuator_health:
                if not self.actuator_health[surface]:
                    effectiveness[surface] = 0.0  # No effectiveness if failed
                else:
                    # Effectiveness decreases near angle limits
                    angle_fraction = abs(angle) / self.xtail_max_angle
                    effectiveness[surface] = max(0.1, 1.0 - 0.3 * angle_fraction)
            
        return effectiveness

    def get_mixing_matrix(self) -> Dict[str, Dict[str, float]]:
        """
        Return the X-tail mixing matrix for analysis/debugging.
        Shows how elevator and rudder commands map to individual surfaces.
        
        Returns:
            Dictionary showing mixing coefficients
        """
        return {
            "port_upper": {"elevator": 1.0, "rudder": 1.0},
            "starboard_upper": {"elevator": 1.0, "rudder": -1.0},
            "port_lower": {"elevator": -1.0, "rudder": 1.0},
            "starboard_lower": {"elevator": -1.0, "rudder": -1.0}
        }

    def reset(self):
        """Reset actuator module to initial state"""
        self.port_upper_angle = 0.0
        self.starboard_upper_angle = 0.0
        self.port_lower_angle = 0.0
        self.starboard_lower_angle = 0.0
        self.thrust_pwm = self.neutral_pwm
        
        # Reset health status
        for actuator in self.actuator_health:
            self.actuator_health[actuator] = True
        
        # Reset rate limiting
        self.last_update_time = None
        for surface in self.previous_angles:
            self.previous_angles[surface] = 0.0
            
        self.logger.info("Actuator module reset to neutral positions")

    def __str__(self) -> str:
        """String representation for debugging"""
        return (f"ActuatorModule("
                f"PU={self.port_upper_angle:.1f}°, "
                f"SU={self.starboard_upper_angle:.1f}°, "
                f"PL={self.port_lower_angle:.1f}°, "
                f"SL={self.starboard_lower_angle:.1f}°, "
                f"PWM={self.thrust_pwm})")

    # Windows 11 compatibility methods
    @staticmethod
    def get_windows_compatible_time():
        """Get time in Windows 11 compatible format"""
        import time
        return time.time()

    def log_windows_safe(self, message: str, level: str = "INFO"):
        """Windows 11 safe logging that handles path issues"""
        try:
            if level == "ERROR":
                self.logger.error(message)
            elif level == "WARNING":
                self.logger.warning(message)
            else:
                self.logger.info(message)
        except Exception:
            # Fallback to print if logging fails on Windows
            print(f"[{level}] ActuatorModule: {message}")

# Windows 11 compatible utility functions
def create_default_actuator_config() -> Dict[str, Any]:
    """Create default configuration suitable for Windows 11 deployment"""
    return {
        "xtail_max_angle": 30.0,
        "neutral_pwm": 1500,
        "pwm_range": 400,
        "max_rate_deg_per_sec": 60.0,
        "windows_compatible": True
    }

def validate_actuator_commands(commands: Dict[str, Any]) -> bool:
    """Validate actuator commands for Windows 11 compatibility"""
    required_keys = ["port_upper_angle", "starboard_upper_angle", 
                    "port_lower_angle", "starboard_lower_angle", "thrust_pwm"]
    
    try:
        # Check all required keys exist
        for key in required_keys:
            if key not in commands:
                return False
        
        # Validate angle ranges
        angle_keys = [k for k in required_keys if k.endswith("_angle")]
        for key in angle_keys:
            angle = float(commands[key])
            if not (-30.0 <= angle <= 30.0):
                return False
        
        # Validate PWM range
        pwm = int(commands["thrust_pwm"])
        if not (1100 <= pwm <= 1900):
            return False
        
        return True
        
    except (ValueError, TypeError):
        return False
        