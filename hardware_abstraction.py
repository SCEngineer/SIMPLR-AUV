#!/usr/bin/env python3
"""
SIMPLR-AUV Hardware Abstraction Layer - CLEAN VERSION
Provides unified interfaces for sensors and actuators in both simulation and real hardware.
FIXED: Proper depth simulation, GPS logic, and ballast integration
"""

import random
import time
import math
from typing import Dict, Any, Optional
from abc import ABC, abstractmethod
from core_types import *

# Global flag to determine if running in simulation mode
USE_SIMULATION = True

# ------------------- SENSOR INTERFACES -------------------

class SensorInterface(ABC):
    """Abstract base class for all sensor interfaces"""
    
    @abstractmethod
    def read(self) -> Dict[str, Any]:
        pass
    
    @abstractmethod
    def is_healthy(self) -> bool:
        pass

class GPSInterface(SensorInterface):
    """GPS sensor interface - FIXED VERSION"""
    
    def __init__(self):
        self.last_fix_time = 0.0
        self.simulation_lat = 34.0001
        self.simulation_lon = -117.0001
    
    def read(self) -> Dict[str, Any]:
        if USE_SIMULATION:
            return self._simulate_gps()
        else:
            return self._read_hardware_gps()
    
    def _simulate_gps(self) -> Dict[str, Any]:
        current_time = time.time()
        
        # FIXED: GPS should only be available at surface (depth < 1m)
        current_depth = getattr(depth_interface, 'simulated_depth', 0.0) if 'depth_interface' in globals() else 0.0
        at_surface = current_depth < 1.0  # GPS only works at surface
        
        if at_surface:
            # Add small random walk to position
            self.simulation_lat += random.uniform(-0.00001, 0.00001)
            self.simulation_lon += random.uniform(-0.00001, 0.00001)
            self.last_fix_time = current_time
            
            return {
                'latitude': self.simulation_lat + random.gauss(0, 0.00003),  # ~3m accuracy
                'longitude': self.simulation_lon + random.gauss(0, 0.00003),
                'altitude': random.uniform(-2, 2),
                'fix_quality': 1,
                'satellites': random.randint(6, 12),
                'timestamp': current_time
            }
        else:
            # No GPS fix underwater
            return {
                'latitude': 0.0,
                'longitude': 0.0,
                'altitude': 0.0,
                'fix_quality': 0,  # No fix underwater
                'satellites': 0,
                'timestamp': current_time
            }
    
    def _read_hardware_gps(self) -> Dict[str, Any]:
        # Placeholder for real GPS hardware interface
        return {
            'latitude': 34.0001,
            'longitude': -117.0001,
            'altitude': 0.0,
            'fix_quality': 1,
            'satellites': 8,
            'timestamp': time.time()
        }
    
    def is_healthy(self) -> bool:
        return True

class IMUInterface(SensorInterface):
    """IMU sensor interface"""
    
    def __init__(self):
        self.bias_x = random.gauss(0, 0.01)
        self.bias_y = random.gauss(0, 0.01)
        self.bias_z = random.gauss(0, 0.01)
    
    def read(self) -> Dict[str, Any]:
        if USE_SIMULATION:
            return self._simulate_imu()
        else:
            return self._read_hardware_imu()
    
    def _simulate_imu(self) -> Dict[str, Any]:
        return {
            'acceleration': [
                random.gauss(0, 0.1),  # X-axis
                random.gauss(0, 0.1),  # Y-axis
                random.gauss(-9.81, 0.1)  # Z-axis (gravity)
            ],
            'angular_velocity': [
                random.gauss(0, 0.05) + self.bias_x,
                random.gauss(0, 0.05) + self.bias_y,
                random.gauss(0, 0.05) + self.bias_z
            ],
            'magnetic_field': [
                random.gauss(0.3, 0.01),
                random.gauss(0.0, 0.01),
                random.gauss(0.5, 0.01)
            ],
            'temperature': random.gauss(20.0, 2.0)
        }
    
    def _read_hardware_imu(self) -> Dict[str, Any]:
        # Placeholder for real IMU hardware interface
        return {
            'acceleration': [0.0, 0.0, -9.81],
            'angular_velocity': [0.0, 0.0, 0.0],
            'magnetic_field': [0.3, 0.0, 0.5],
            'temperature': 20.0
        }
    
    def is_healthy(self) -> bool:
        return True

class DepthInterface(SensorInterface):
    """Depth/Pressure sensor interface - FIXED VERSION WITH PROPER BALLAST INTEGRATION"""
    
    def __init__(self):
        self.simulated_depth = 0.0  # FIXED: Start at surface!
        self.target_depth = 0.0
        self.depth_rate = 0.3  # m/s descent/ascent rate
        self.last_update = time.time()
    
    def read(self) -> Dict[str, Any]:
        if USE_SIMULATION:
            return self._simulate_depth()
        else:
            return self._read_hardware_depth()
    
    def _simulate_depth(self) -> Dict[str, Any]:
        current_time = time.time()
        dt = min(current_time - self.last_update, 0.2)  # Cap dt to prevent jumps
        self.last_update = current_time
        
        # FIXED: Get ballast fill level from the global ballast controller
        ballast_fill = 0.0
        if 'ballast_controller' in globals() and ballast_controller:
            ballast_fill = getattr(ballast_controller, 'fill_level', 0.0)
        
        # FIXED: More aggressive depth response to ballast changes
        if ballast_fill > 0.8:  # Tank mostly full = sink aggressively
            self.target_depth = 8.0  # Deeper target for full ballast
            self.depth_rate = 0.8    # Faster sinking rate
        elif ballast_fill > 0.5:  # Tank half full = moderate sink
            self.target_depth = 3.0
            self.depth_rate = 0.5
        elif ballast_fill < 0.2:  # Tank mostly empty = surface
            self.target_depth = 0.1
            self.depth_rate = 0.6    # Fast ascent rate
        else:
            self.target_depth = 1.0  # Neutral depth
            self.depth_rate = 0.3
        
        # Move toward target depth
        depth_error = self.target_depth - self.simulated_depth
        if abs(depth_error) > 0.05:
            direction = 1.0 if depth_error > 0 else -1.0
            change = direction * self.depth_rate * dt
            self.simulated_depth += change
            
            # Debug output for ballast-depth relationship
            if int(current_time) % 5 == 0:  # Every 5 seconds
                print(f"[DEPTH] Ballast: {ballast_fill*100:.0f}%, Target: {self.target_depth:.1f}m, Current: {self.simulated_depth:.1f}m")
        
        # Enforce physical limits
        self.simulated_depth = max(0.0, min(20.0, self.simulated_depth))
        
        pressure = self.simulated_depth * 9810  # More accurate pressure in Pa
        
        return {
            'depth_m': self.simulated_depth + random.gauss(0, 0.01),  # 1cm accuracy
            'pressure_pa': pressure + random.gauss(0, 100),
            'temperature': random.gauss(15.0, 1.0)
        }
    
    def _read_hardware_depth(self) -> Dict[str, Any]:
        # Placeholder for real depth sensor interface
        return {
            'depth_m': 0.0,  # Start at surface
            'pressure_pa': 101325,  # Sea level pressure
            'temperature': 15.0
        }
    
    def set_target_depth(self, depth: float):
        """Called by ballast system to update target depth"""
        self.target_depth = max(0.0, min(20.0, depth))
    
    def is_healthy(self) -> bool:
        return True

class BatteryInterface(SensorInterface):
    """Battery monitoring interface"""
    
    def __init__(self):
        self.capacity = 100.0
        self.discharge_rate = 0.0005  # %/second (slower discharge)
        self.start_time = time.time()
    
    def read(self) -> Dict[str, Any]:
        if USE_SIMULATION:
            return self._simulate_battery()
        else:
            return self._read_hardware_battery()
    
    def _simulate_battery(self) -> Dict[str, Any]:
        # Simulate battery discharge over time
        elapsed = time.time() - self.start_time
        self.capacity = max(0.0, 100.0 - elapsed * self.discharge_rate)
        
        voltage = 11.5 + (self.capacity / 100.0) * 1.0  # 11.5V to 12.5V range
        current = random.gauss(1.5, 0.2)
        
        return {
            'voltage': voltage + random.gauss(0, 0.05),
            'current': current,
            'capacity_remaining': self.capacity,
            'power': voltage * current
        }
    
    def _read_hardware_battery(self) -> Dict[str, Any]:
        # Placeholder for real battery monitoring
        return {
            'voltage': 12.1,
            'current': 1.3,
            'capacity_remaining': 85.0,
            'power': 15.73
        }
    
    def is_healthy(self) -> bool:
        return True

class LeakDetectorInterface(SensorInterface):
    """Leak detection interface"""
    
    def read(self) -> Dict[str, Any]:
        if USE_SIMULATION:
            return self._simulate_leak_detector()
        else:
            return self._read_hardware_leak_detector()
    
    def _simulate_leak_detector(self) -> Dict[str, Any]:
        return {
            'leak_detected': False,
            'water_level': random.uniform(0.0, 0.001)
        }
    
    def _read_hardware_leak_detector(self) -> Dict[str, Any]:
        return {
            'leak_detected': False,
            'water_level': 0.0
        }
    
    def is_healthy(self) -> bool:
        return True

class WaterSpeedInterface(SensorInterface):
    """Water speed sensor interface (paddlewheel or DVL)"""
    
    def read(self) -> Dict[str, Any]:
        if USE_SIMULATION:
            return self._simulate_water_speed()
        else:
            return self._read_hardware_water_speed()
    
    def _simulate_water_speed(self) -> Dict[str, Any]:
        # Simulate realistic water speed based on depth and vehicle state
        current_depth = getattr(depth_interface, 'simulated_depth', 0.0) if 'depth_interface' in globals() else 0.0
        
        if current_depth > 0.5:  # Underwater
            return {
                'vx': random.gauss(0.8, 0.1),  # Forward velocity
                'vy': random.gauss(0.0, 0.05),  # Lateral velocity
                'vz': random.gauss(0.0, 0.02)   # Vertical velocity
            }
        else:  # At surface
            return {
                'vx': random.gauss(0.2, 0.05),  # Slower at surface
                'vy': random.gauss(0.0, 0.02),
                'vz': random.gauss(0.0, 0.01)
            }
    
    def _read_hardware_water_speed(self) -> Dict[str, Any]:
        return {
            'vx': 1.0,
            'vy': 0.0,
            'vz': 0.0
        }
    
    def is_healthy(self) -> bool:
        return True

# ------------------- ACTUATOR INTERFACES -------------------

class ActuatorInterface(ABC):
    """Abstract base class for all actuator interfaces"""
    
    @abstractmethod
    def set_command(self, command: Any) -> bool:
        pass
    
    @abstractmethod
    def is_healthy(self) -> bool:
        pass

class ServoController(ActuatorInterface):
    """Servo controller for control surfaces"""
    
    def __init__(self):
        self.servo_positions = {
            'port_upper': 0.0,
            'starboard_upper': 0.0,
            'port_lower': 0.0,
            'starboard_lower': 0.0
        }
    
    def set_angle(self, servo_id: str, angle_degrees: float) -> bool:
        """Set servo angle in degrees"""
        if USE_SIMULATION:
            return self._simulate_servo(servo_id, angle_degrees)
        else:
            return self._set_hardware_servo(servo_id, angle_degrees)
    
    def _simulate_servo(self, servo_id: str, angle_degrees: float) -> bool:
        # Clamp angle to valid range
        angle_degrees = max(-30.0, min(30.0, angle_degrees))
        if servo_id in self.servo_positions:
            self.servo_positions[servo_id] = angle_degrees
            return True
        return False
    
    def _set_hardware_servo(self, servo_id: str, angle_degrees: float) -> bool:
        # Placeholder for real servo control
        return True
    
    def set_command(self, command: Dict[str, float]) -> bool:
        """Set multiple servo positions"""
        success = True
        for servo_id, angle in command.items():
            if not self.set_angle(servo_id, angle):
                success = False
        return success
    
    def is_healthy(self) -> bool:
        return True

class ThrusterController(ActuatorInterface):
    """Thruster controller"""
    
    def __init__(self):
        self.current_pwm = 1500  # Neutral
    
    def set_pwm(self, pwm_value: int) -> bool:
        """Set thruster PWM (1100-1900 microseconds)"""
        if USE_SIMULATION:
            return self._simulate_thruster(pwm_value)
        else:
            return self._set_hardware_thruster(pwm_value)
    
    def _simulate_thruster(self, pwm_value: int) -> bool:
        # Clamp PWM to valid range
        pwm_value = max(1100, min(1900, pwm_value))
        self.current_pwm = pwm_value
        return True
    
    def _set_hardware_thruster(self, pwm_value: int) -> bool:
        # Placeholder for real thruster control
        return True
    
    def set_command(self, command: int) -> bool:
        return self.set_pwm(command)
    
    def is_healthy(self) -> bool:
        return True

class BallastController(ActuatorInterface):
    """Ballast system controller - FIXED VERSION with proper simulation"""
    
    def __init__(self):
        self.valve_state = "CLOSED"
        self.pump_state = "OFF"
        self.fill_level = 0.0  # FIXED: Track actual fill level (0.0 = empty, 1.0 = full)
        self.fill_rate = 0.03  # 3% per second
        self.empty_rate = 0.04  # 4% per second (faster empty)
        self.last_update = time.time()
    
    def set_valve(self, state: str) -> bool:
        """Set valve state (OPEN/CLOSED)"""
        if USE_SIMULATION:
            return self._simulate_valve(state)
        else:
            return self._set_hardware_valve(state)
    
    def set_pump(self, state: str) -> bool:
        """Set pump state (ON/OFF)"""
        if USE_SIMULATION:
            return self._simulate_pump(state)
        else:
            return self._set_hardware_pump(state)
    
    def _simulate_valve(self, state: str) -> bool:
        if state in ["OPEN", "CLOSED"]:
            self.valve_state = state
            return True
        return False
    
    def _simulate_pump(self, state: str) -> bool:
        if state in ["ON", "OFF"]:
            self.pump_state = state
            return True
        return False
    
    def _set_hardware_valve(self, state: str) -> bool:
        # Placeholder for real valve control
        return True
    
    def _set_hardware_pump(self, state: str) -> bool:
        # Placeholder for real pump control
        return True
    
    def update_ballast_simulation(self, dt: float = None):
        """FIXED: Update ballast fill level based on commands"""
        current_time = time.time()
        if dt is None:
            dt = min(current_time - self.last_update, 0.2)  # Cap dt
        self.last_update = current_time
        
        if self.pump_state == "ON":
            if self.valve_state == "CLOSED":
                # Filling tank
                self.fill_level += self.fill_rate * dt
                self.fill_level = min(1.0, self.fill_level)
            elif self.valve_state == "OPEN":
                # Emptying tank (or emergency blow)
                self.fill_level -= self.empty_rate * dt
                self.fill_level = max(0.0, self.fill_level)
    
    def set_command(self, command: Dict[str, str]) -> bool:
        success = True
        if 'valve' in command:
            if not self.set_valve(command['valve']):
                success = False
        if 'pump' in command:
            if not self.set_pump(command['pump']):
                success = False
        return success
    
    def is_healthy(self) -> bool:
        return True

class RCReceiver:
    """RC receiver interface for manual override"""
    
    def __init__(self):
        self.channels = {f'ch{i}': 1500 for i in range(1, 9)}
        self.signal_valid = False
        self.last_update = 0.0
    
    def read_all_channels(self) -> Dict[str, int]:
        """Read all RC channels"""
        if USE_SIMULATION:
            return self._simulate_rc()
        else:
            return self._read_hardware_rc()
    
    def _simulate_rc(self) -> Dict[str, int]:
        # Simulate occasional RC signal for testing
        current_time = time.time()
        if (current_time % 300) < 10:  # Signal for 10s every 5 minutes
            self.signal_valid = True
            self.last_update = current_time
            # Simulate some stick movement
            self.channels['ch1'] = 1500 + int(100 * math.sin(current_time))
            self.channels['ch8'] = 2000  # Override switch on
        else:
            self.signal_valid = False
        
        return self.channels.copy()
    
    def _read_hardware_rc(self) -> Dict[str, int]:
        # Placeholder for real RC receiver
        return self.channels.copy()
    
    def is_signal_valid(self) -> bool:
        """Check if RC signal is valid"""
        if USE_SIMULATION:
            return self.signal_valid
        else:
            # Check signal timeout
            return (time.time() - self.last_update) < 1.0
    
    def is_override_active(self) -> bool:
        """Check if manual override is active"""
        if not self.is_signal_valid():
            return False
        return self.channels.get('ch8', 1000) > 1700  # Override switch threshold

# ------------------- HARDWARE ABSTRACTION LAYER -------------------

# Initialize all hardware interfaces
gps_interface = GPSInterface()
imu_interface = IMUInterface()
depth_interface = DepthInterface()
battery_interface = BatteryInterface()
leak_detector_interface = LeakDetectorInterface()
water_speed_interface = WaterSpeedInterface()

servo_controller = ServoController()
thruster_controller = ThrusterController()
ballast_controller = BallastController()
rc_receiver = RCReceiver()

def read_sensors() -> Dict[str, Any]:
    """
    FIXED: Unified sensor reading interface with proper ballast integration
    """
    try:
        # CRITICAL: Update ballast simulation first
        ballast_controller.update_ballast_simulation()
        
        sensor_data = {
            'gps': gps_interface.read(),
            'imu': imu_interface.read(),
            'depth': depth_interface.read(),  # This will now use updated ballast data
            'battery': battery_interface.read(),
            'leak_detector': leak_detector_interface.read(),
            'water_speed': water_speed_interface.read(),
            'ballast': {
                'fill_level': ballast_controller.fill_level,
                'is_full': ballast_controller.fill_level > 0.95,
                'is_empty': ballast_controller.fill_level < 0.05,
                'valve_state': ballast_controller.valve_state,
                'pump_state': ballast_controller.pump_state,
                'state': 'FULL' if ballast_controller.fill_level > 0.95 else 
                        'EMPTY' if ballast_controller.fill_level < 0.05 else
                        'FILLING' if ballast_controller.pump_state == 'ON' and ballast_controller.valve_state == 'CLOSED' else
                        'EMPTYING' if ballast_controller.pump_state == 'ON' and ballast_controller.valve_state == 'OPEN' else
                        'UNKNOWN'
            },
            'system': {
                'timestamp': time.time(),
                'health_status': 'HEALTHY',
                'diagnostic_code': 0,
                'error_message': ''
            }
        }
        return sensor_data
    except Exception as e:
        return {
            'gps': {'latitude': 34.0001, 'longitude': -117.0001, 'fix_quality': 0},
            'imu': {'acceleration': [0.0, 0.0, -9.81], 'angular_velocity': [0.0, 0.0, 0.0], 'magnetic_field': [0.3, 0.0, 0.5]},
            'depth': {'depth_m': 0.0, 'pressure_pa': 101325, 'temperature': 15.0},
            'battery': {'voltage': 12.0, 'current': 0.0, 'capacity_remaining': 100.0, 'power': 0.0},
            'leak_detector': {'leak_detected': False, 'water_level': 0.0},
            'water_speed': {'vx': 0.0, 'vy': 0.0, 'vz': 0.0},
            'ballast': {'fill_level': 0.0, 'is_full': False, 'is_empty': True, 'state': 'EMPTY'},
            'system': {
                'timestamp': time.time(),
                'health_status': 'ERROR',
                'diagnostic_code': 500,
                'error_message': str(e)
            }
        }

def send_actuator_commands(commands: Dict[str, Any]) -> bool:
    """
    FIXED: Unified actuator command interface with proper ballast handling
    """
    try:
        # CRITICAL: Check for RC override FIRST - safety critical
        if rc_receiver.is_signal_valid() and rc_receiver.is_override_active():
            return _apply_manual_control()
        
        # Autonomous control - apply computed commands
        success = True
        
        # Control surface commands
        if any(key in commands for key in ['port_upper_angle', 'starboard_upper_angle', 
                                           'port_lower_angle', 'starboard_lower_angle']):
            servo_commands = {
                key.replace('_angle', ''): commands.get(key, 0.0)
                for key in ['port_upper_angle', 'starboard_upper_angle', 
                           'port_lower_angle', 'starboard_lower_angle']
                if key in commands
            }
            if not servo_controller.set_command(servo_commands):
                success = False
        
        # Thruster commands
        if 'thrust_pwm' in commands:
            if not thruster_controller.set_pwm(int(commands['thrust_pwm'])):
                success = False
        
        # FIXED: Ballast commands with proper mapping
        if 'ballast_command' in commands:
            ballast_cmd = commands['ballast_command']
            if ballast_cmd == 'FILL':
                ballast_controller.set_valve('CLOSED')
                ballast_controller.set_pump('ON')
            elif ballast_cmd == 'EMPTY' or ballast_cmd == 'EMERGENCY_BLOW':
                ballast_controller.set_valve('OPEN')
                ballast_controller.set_pump('ON')
            elif ballast_cmd == 'HOLD':
                ballast_controller.set_pump('OFF')
            
        # Legacy ballast commands
        if 'ballast_valve' in commands:
            ballast_controller.set_valve(commands['ballast_valve'])
        if 'ballast_pump' in commands:
            ballast_controller.set_pump(commands['ballast_pump'])
        
        return success
        
    except Exception as e:
        # Emergency safe state on actuator failure
        _emergency_safe_actuator_state()
        raise Exception(f"Actuator control failed: {e}")

def _apply_manual_control() -> bool:
    """Apply manual RC control commands"""
    try:
        rc_channels = rc_receiver.read_all_channels()
        
        # Convert RC PWM to servo angles (1000-2000 PWM -> -30 to +30 degrees)
        def pwm_to_angle(pwm: int) -> float:
            return (pwm - 1500) * 30.0 / 500.0
        
        # Apply manual servo commands
        servo_commands = {
            'port_upper': pwm_to_angle(rc_channels.get('ch1', 1500)),
            'starboard_upper': pwm_to_angle(rc_channels.get('ch2', 1500)),
            'port_lower': pwm_to_angle(rc_channels.get('ch3', 1500)),
            'starboard_lower': pwm_to_angle(rc_channels.get('ch4', 1500))
        }
        servo_controller.set_command(servo_commands)
        
        # Apply manual thruster command
        thruster_pwm = rc_channels.get('ch5', 1500)
        thruster_controller.set_pwm(thruster_pwm)
        
        # Apply manual ballast command (3-position switch)
        ballast_pwm = rc_channels.get('ch6', 1500)
        if ballast_pwm < 1300:  # Empty
            ballast_controller.set_valve("OPEN")
            ballast_controller.set_pump("ON")
        elif ballast_pwm > 1700:  # Fill
            ballast_controller.set_valve("CLOSED")
            ballast_controller.set_pump("ON")
        else:  # Hold
            ballast_controller.set_valve("CLOSED")
            ballast_controller.set_pump("OFF")
        
        # Check emergency surface command
        if rc_channels.get('ch7', 1000) > 1700:
            _emergency_surface_command()
        
        return True
        
    except Exception as e:
        print(f"Manual control error: {e}")
        return False

def _emergency_safe_actuator_state():
    """Set all actuators to safe state"""
    # Center all control surfaces
    servo_controller.set_command({
        'port_upper': 0.0,
        'starboard_upper': 0.0,
        'port_lower': 0.0,
        'starboard_lower': 0.0
    })
    
    # Stop thruster
    thruster_controller.set_pwm(1500)
    
    # Emergency blow ballast
    ballast_controller.set_valve("OPEN")
    ballast_controller.set_pump("ON")

def _emergency_surface_command():
    """Execute emergency surface procedure"""
    # Emergency blow ballast
    ballast_controller.set_valve("OPEN")
    ballast_controller.set_pump("ON")
    
    # Center control surfaces
    servo_controller.set_command({
        'port_upper': 0.0,
        'starboard_upper': 0.0,
        'port_lower': 0.0,
        'starboard_lower': 0.0
    })
    
    # Minimal forward thrust
    thruster_controller.set_pwm(1600)

def set_simulation_mode(use_simulation: bool):
    """Set whether to use simulation or real hardware"""
    global USE_SIMULATION
    USE_SIMULATION = use_simulation

def get_hardware_status() -> Dict[str, bool]:
    """Get health status of all hardware components"""
    return {
        'gps': gps_interface.is_healthy(),
        'imu': imu_interface.is_healthy(),
        'depth': depth_interface.is_healthy(),
        'battery': battery_interface.is_healthy(),
        'leak_detector': leak_detector_interface.is_healthy(),
        'water_speed': water_speed_interface.is_healthy(),
        'servos': servo_controller.is_healthy(),
        'thruster': thruster_controller.is_healthy(),
        'ballast': ballast_controller.is_healthy(),
        'rc_receiver': rc_receiver.is_signal_valid()
    }

def initialize_hardware() -> bool:
    """Initialize all hardware interfaces"""
    try:
        # Perform hardware initialization checks
        status = get_hardware_status()
        
        # Log initialization status
        print("Hardware Initialization Status:")
        for component, healthy in status.items():
            status_str = "OK" if healthy else "FAULT"
            print(f"  {component}: {status_str}")
        
        # Set safe initial state
        _emergency_safe_actuator_state()
        
        # FIXED: Initialize ballast to empty state
        ballast_controller.fill_level = 0.0  # Start empty
        ballast_controller.valve_state = "CLOSED"
        ballast_controller.pump_state = "OFF"
        
        return all(status.values())
        
    except Exception as e:
        print(f"Hardware initialization failed: {e}")
        return False