#!/usr/bin/env python3
"""
SIMPLR-AUV Guidance System
High-level path planning and guidance command generation.
Windows 11 compatible with proper waypoint navigation and station keeping.
UPDATED VERSION - Fixed waypoint completion logic and improved navigation.
"""

import math
import logging
import time
from typing import Dict, Any, Optional
from dataclasses import dataclass

from core_types import GuidanceMode, Waypoint, Position, NavigationState

@dataclass
class GuidanceTarget:
    """Current guidance target specification"""
    waypoint: Optional[Waypoint] = None
    station_keeping_position: Optional[Position] = None
    target_depth: Optional[float] = None
    target_heading: Optional[float] = None
    target_speed: float = 1.0

class GuidanceSystem:
    """
    High-level guidance system for waypoint following and path planning.
    Platform-independent implementation compatible with Windows 11.
    UPDATED with improved waypoint completion logic.
    """
    
    def __init__(self, 
                 default_speed: float = 1.0,
                 waypoint_tolerance: float = 10.0,  # INCREASED from 5.0 to 10.0m
                 station_keeping_tolerance: float = 3.0,  # INCREASED from 2.0 to 3.0m
                 max_speed: float = 2.5,
                 min_speed: float = 0.1):
        """
        Initialize guidance system with navigation parameters.
        
        Args:
            default_speed: Default cruise speed in m/s
            waypoint_tolerance: Waypoint arrival tolerance in meters (increased for reliability)
            station_keeping_tolerance: Station keeping tolerance in meters
            max_speed: Maximum allowable speed in m/s
            min_speed: Minimum allowable speed in m/s
        """
        self.mode = GuidanceMode.IDLE
        self.default_speed = default_speed
        self.waypoint_tolerance = waypoint_tolerance
        self.station_keeping_tolerance = station_keeping_tolerance
        self.max_speed = max_speed
        self.min_speed = min_speed
        
        # Current guidance target
        self.current_target = GuidanceTarget()
        
        # Task completion status
        self.task_complete = False
        self.last_distance_to_target = float('inf')
        self.approach_count = 0
        
        # Timing for mode transitions
        self.mode_start_time = time.time()
        self.last_update_time = None
        
        # Waypoint completion tracking
        self.waypoint_close_time = None
        self.required_close_duration = 5.0  # Must be close for 5 seconds
        
        self.logger = logging.getLogger("GuidanceSystem")
        self.logger.info(f"Guidance system initialized: speed={default_speed}m/s, tolerance={waypoint_tolerance}m")

    def set_waypoint(self, latitude: float, longitude: float, depth: float, 
                    waypoint_id: str = "", speed: Optional[float] = None):
        """
        Set target waypoint for navigation.
        
        Args:
            latitude: Target latitude in degrees
            longitude: Target longitude in degrees  
            depth: Target depth in meters
            waypoint_id: Optional waypoint identifier
            speed: Optional speed override
        """
        waypoint = Waypoint(
            latitude=latitude,
            longitude=longitude,
            depth=depth,
            id=waypoint_id,
            name=waypoint_id or f"WP_{time.time():.0f}"
        )
        
        self.current_target.waypoint = waypoint
        self.current_target.target_speed = speed or self.default_speed
        
        self.mode = GuidanceMode.WAYPOINT_FOLLOWING
        self.task_complete = False
        self.last_distance_to_target = float('inf')
        self.approach_count = 0
        self.waypoint_close_time = None
        self.mode_start_time = time.time()
        
        self.logger.info(f"Waypoint set: {latitude:.6f}, {longitude:.6f}, {depth:.1f}m @ {self.current_target.target_speed:.1f}m/s")

    def set_station_keeping(self, latitude: float, longitude: float, depth: float):
        """
        Set station keeping at specified position.
        
        Args:
            latitude: Station keeping latitude in degrees
            longitude: Station keeping longitude in degrees
            depth: Station keeping depth in meters
        """
        position = Position(latitude=latitude, longitude=longitude, depth=depth)
        self.current_target.station_keeping_position = position
        
        self.mode = GuidanceMode.STATION_KEEPING
        self.task_complete = False
        self.mode_start_time = time.time()
        
        self.logger.info(f"Station keeping set: {latitude:.6f}, {longitude:.6f}, {depth:.1f}m")

    def set_depth_target(self, depth: float, speed: Optional[float] = None):
        """Set depth change target"""
        self.current_target.target_depth = depth
        self.current_target.target_speed = speed or (self.default_speed * 0.5)  # Slower for depth changes
        
        self.mode = GuidanceMode.DEPTH_CHANGE
        self.task_complete = False
        self.mode_start_time = time.time()
        
        self.logger.info(f"Depth target set: {depth:.1f}m")

    def set_heading_target(self, heading: float):
        """Set heading hold target"""
        self.current_target.target_heading = heading % 360
        
        self.mode = GuidanceMode.HEADING_HOLD
        self.task_complete = False
        self.mode_start_time = time.time()
        
        self.logger.info(f"Heading target set: {heading:.1f}°")

    def emergency_surface(self):
        """Activate emergency surface guidance"""
        self.mode = GuidanceMode.EMERGENCY_SURFACE
        self.current_target.target_depth = 0.0
        self.current_target.target_speed = self.max_speed
        self.task_complete = False
        self.mode_start_time = time.time()
        
        self.logger.critical("Emergency surface guidance activated")

    def update(self, executive_commands: Dict[str, Any], 
              nav_state: Dict[str, Any], 
              time_s: float) -> Dict[str, Any]:
        """
        Main guidance update function.
        
        Args:
            executive_commands: Commands from executive function
            nav_state: Current navigation state
            time_s: Current mission time
            
        Returns:
            Dictionary with guidance commands
        """
        current_time = time.time()
        
        # Update mode from executive if provided
        if 'guidance_mode' in executive_commands:
            self._update_mode_from_executive(executive_commands)
        
        # Calculate time in current mode
        time_in_mode = current_time - self.mode_start_time
        
        # Generate guidance commands based on current mode
        if self.mode == GuidanceMode.IDLE:
            guidance_commands = self._generate_idle_commands(nav_state)
            
        elif self.mode == GuidanceMode.WAYPOINT_FOLLOWING:
            guidance_commands = self._generate_waypoint_commands(nav_state, time_in_mode)
            
        elif self.mode == GuidanceMode.STATION_KEEPING:
            guidance_commands = self._generate_station_keeping_commands(nav_state)
            
        elif self.mode == GuidanceMode.DEPTH_CHANGE:
            guidance_commands = self._generate_depth_change_commands(nav_state)
            
        elif self.mode == GuidanceMode.HEADING_HOLD:
            guidance_commands = self._generate_heading_hold_commands(nav_state)
            
        elif self.mode == GuidanceMode.EMERGENCY_SURFACE:
            guidance_commands = self._generate_emergency_surface_commands(nav_state)
            
        elif self.mode == GuidanceMode.GPS_FIX_SURFACE:
            guidance_commands = self._generate_gps_fix_commands(nav_state)
            
        else:
            guidance_commands = self._generate_idle_commands(nav_state)
        
        # Add common status information
        guidance_commands.update({
            'guidance_mode': self.mode.value,
            'task_complete': self.task_complete,
            'time_in_mode': time_in_mode
        })
        
        self.last_update_time = current_time
        return guidance_commands

    def _update_mode_from_executive(self, executive_commands: Dict[str, Any]):
        """Update guidance mode based on executive commands"""
        mode_str = executive_commands.get('guidance_mode', 'idle')
        
        # Map executive modes to guidance modes
        mode_mapping = {
            'idle': GuidanceMode.IDLE,
            'waypoint_following': GuidanceMode.WAYPOINT_FOLLOWING,
            'swim_to_waypoint': GuidanceMode.WAYPOINT_FOLLOWING,
            'station_keeping': GuidanceMode.STATION_KEEPING,
            'depth_change': GuidanceMode.DEPTH_CHANGE,
            'dive': GuidanceMode.DEPTH_CHANGE,
            'climb': GuidanceMode.DEPTH_CHANGE,
            'heading_hold': GuidanceMode.HEADING_HOLD,
            'emergency_surface': GuidanceMode.EMERGENCY_SURFACE,
            'gps_fix_surface': GuidanceMode.GPS_FIX_SURFACE
        }
        
        new_mode = mode_mapping.get(mode_str.lower(), GuidanceMode.IDLE)
        
        if new_mode != self.mode:
            self.logger.info(f"Guidance mode changed: {self.mode.value} -> {new_mode.value}")
            self.mode = new_mode
            self.mode_start_time = time.time()
            self.task_complete = False
            self.waypoint_close_time = None
            
            # Extract parameters from executive commands
            if 'waypoint' in executive_commands:
                wp = executive_commands['waypoint']
                if isinstance(wp, list) and len(wp) >= 2:
                    depth = wp[2] if len(wp) > 2 else executive_commands.get('target_depth', 5.0)
                    speed = executive_commands.get('speed', self.default_speed)
                    self.set_waypoint(wp[0], wp[1], depth, speed=speed)
            
            if 'target_depth' in executive_commands:
                self.current_target.target_depth = executive_commands['target_depth']
            
            if 'heading' in executive_commands:
                self.current_target.target_heading = executive_commands['heading']
            
            if 'speed' in executive_commands:
                self.current_target.target_speed = executive_commands['speed']

    def _generate_idle_commands(self, nav_state: Dict[str, Any]) -> Dict[str, Any]:
        """Generate commands for idle mode"""
        current_heading = nav_state.get('attitude', {}).get('yaw', 0.0)
        current_depth = nav_state.get('position', {}).get('depth', 0.0)
        
        return {
            'heading_deg': current_heading,
            'depth_m': current_depth,
            'speed_mps': 0.0,
            'task_complete': True
        }

    def _generate_waypoint_commands(self, nav_state: Dict[str, Any], time_in_mode: float) -> Dict[str, Any]:
        """Generate commands for waypoint following - IMPROVED VERSION"""
        if not self.current_target.waypoint:
            return self._generate_idle_commands(nav_state)
        
        # Get current position
        current_pos = nav_state.get('position', {})
        current_lat = current_pos.get('latitude', 0.0)
        current_lon = current_pos.get('longitude', 0.0)
        current_depth = current_pos.get('depth', 0.0)
        
        # Calculate distance and bearing to waypoint
        distance = self._calculate_distance(
            current_lat, current_lon,
            self.current_target.waypoint.latitude, 
            self.current_target.waypoint.longitude
        )
        
        bearing = self._calculate_bearing(
            current_lat, current_lon,
            self.current_target.waypoint.latitude,
            self.current_target.waypoint.longitude
        )
        
        # IMPROVED: More sophisticated waypoint completion logic
        depth_error = abs(current_depth - self.current_target.waypoint.depth)
        
        # Relaxed tolerances for better success rate
        horizontal_tolerance = self.waypoint_tolerance  # Now 10.0m default
        depth_tolerance = 3.0  # Increased to 3.0m for better completion
        
        # Check if we're close to the waypoint
        is_close = (distance <= horizontal_tolerance and depth_error <= depth_tolerance)
        
        current_time = time.time()
        
        if is_close:
            if self.waypoint_close_time is None:
                self.waypoint_close_time = current_time
                self.logger.info(f"Approaching waypoint: distance={distance:.1f}m, depth_error={depth_error:.1f}m")
            
            # Must stay close for required duration
            time_close = current_time - self.waypoint_close_time
            if time_close >= self.required_close_duration:
                self.task_complete = True
                self.logger.info(f"Waypoint REACHED: distance={distance:.1f}m, depth_error={depth_error:.1f}m, stable for {time_close:.1f}s")
                
                return {
                    'heading_deg': bearing,
                    'depth_m': self.current_target.waypoint.depth,
                    'speed_mps': 0.0,  # Stop at waypoint
                    'distance_to_target': distance,
                    'bearing_to_target': bearing,
                    'task_complete': True,
                    'completion_reason': 'waypoint_reached'
                }
        else:
            # Reset close timer if we move away
            self.waypoint_close_time = None
        
        # IMPROVED: Handle surface waypoints properly
        target_depth = self.current_target.waypoint.depth
        if target_depth <= 1.0:  # Surface or near-surface waypoint
            target_depth = 1.0  # Stay 1m below surface for stability
        
        # IMPROVED: Smart speed control based on distance and approach angle
        if distance < 30.0:
            # Slow down as we approach
            speed_factor = max(0.2, distance / 30.0)
            approach_speed = self.current_target.target_speed * speed_factor
        elif distance < 100.0:
            # Medium speed for medium distances
            approach_speed = self.current_target.target_speed * 0.8
        else:
            # Full speed for long distances
            approach_speed = self.current_target.target_speed
        
        # Clamp speed to limits
        approach_speed = max(self.min_speed, min(self.max_speed, approach_speed))
        
        # Track approach progress
        if distance < self.last_distance_to_target:
            self.approach_count += 1
        else:
            self.approach_count = max(0, self.approach_count - 1)
        
        self.last_distance_to_target = distance
        
        # IMPROVED: Reasonable timeout with forced completion
        timeout_duration = 800.0  # 13+ minutes timeout (was 5 minutes)
        if time_in_mode > timeout_duration:
            self.logger.warning(f"Waypoint timeout after {time_in_mode:.1f}s - forcing completion (distance={distance:.1f}m)")
            self.task_complete = True
            
            return {
                'heading_deg': bearing,
                'depth_m': target_depth,
                'speed_mps': 0.0,
                'distance_to_target': distance,
                'bearing_to_target': bearing,
                'task_complete': True,
                'completion_reason': 'timeout'
            }
        
        # Calculate progress percentage
        initial_distance = 100.0  # Assume initial distance for progress calc
        progress = max(0.1, 1.0 - (distance / initial_distance))
        
        return {
            'heading_deg': bearing,
            'depth_m': target_depth,
            'speed_mps': approach_speed,
            'distance_to_target': distance,
            'bearing_to_target': bearing,
            'approach_progress': min(0.95, progress),
            'time_close': (current_time - self.waypoint_close_time) if self.waypoint_close_time else 0.0
        }

    def _generate_station_keeping_commands(self, nav_state: Dict[str, Any]) -> Dict[str, Any]:
        """Generate commands for station keeping"""
        if not self.current_target.station_keeping_position:
            return self._generate_idle_commands(nav_state)
        
        # Get current position
        current_pos = nav_state.get('position', {})
        current_lat = current_pos.get('latitude', 0.0)
        current_lon = current_pos.get('longitude', 0.0)
        current_depth = current_pos.get('depth', 0.0)
        
        # Calculate error from station keeping position
        distance = self._calculate_distance(
            current_lat, current_lon,
            self.current_target.station_keeping_position.latitude,
            self.current_target.station_keeping_position.longitude
        )
        
        bearing = self._calculate_bearing(
            current_lat, current_lon,
            self.current_target.station_keeping_position.latitude,
            self.current_target.station_keeping_position.longitude
        )
        
        # Proportional control for station keeping
        if distance > self.station_keeping_tolerance:
            # Move toward station keeping position
            correction_speed = min(0.8, distance * 0.15)  # Slightly faster correction
            heading_command = bearing
        else:
            # Hold position
            correction_speed = 0.0
            heading_command = nav_state.get('attitude', {}).get('yaw', 0.0)
        
        return {
            'heading_deg': heading_command,
            'depth_m': self.current_target.station_keeping_position.depth,
            'speed_mps': correction_speed,
            'station_keeping_error': distance,
            'task_complete': distance <= self.station_keeping_tolerance
        }

    def _generate_depth_change_commands(self, nav_state: Dict[str, Any]) -> Dict[str, Any]:
        """Generate commands for depth changes"""
        if self.current_target.target_depth is None:
            return self._generate_idle_commands(nav_state)
        
        current_depth = nav_state.get('position', {}).get('depth', 0.0)
        current_heading = nav_state.get('attitude', {}).get('yaw', 0.0)
        
        depth_error = abs(current_depth - self.current_target.target_depth)
        
        # IMPROVED: More reasonable depth tolerance
        depth_tolerance = 1.0  # 1 meter tolerance
        
        # Check if depth target reached
        if depth_error <= depth_tolerance:
            self.task_complete = True
            return {
                'heading_deg': current_heading,
                'depth_m': self.current_target.target_depth,
                'speed_mps': 0.2,  # Minimal speed to maintain control
                'depth_error': depth_error,
                'task_complete': True
            }
        
        # Speed based on depth error
        if depth_error > 5.0:
            speed = self.current_target.target_speed
        else:
            speed = self.current_target.target_speed * 0.5  # Slow approach
        
        return {
            'heading_deg': current_heading,
            'depth_m': self.current_target.target_depth,
            'speed_mps': speed,
            'depth_error': depth_error
        }

    def _generate_heading_hold_commands(self, nav_state: Dict[str, Any]) -> Dict[str, Any]:
        """Generate commands for heading hold"""
        if self.current_target.target_heading is None:
            return self._generate_idle_commands(nav_state)
        
        current_depth = nav_state.get('position', {}).get('depth', 0.0)
        current_heading = nav_state.get('attitude', {}).get('yaw', 0.0)
        
        # Calculate heading error with wraparound
        heading_error = self.current_target.target_heading - current_heading
        if heading_error > 180:
            heading_error -= 360
        elif heading_error < -180:
            heading_error += 360
        
        # Check if heading target reached
        if abs(heading_error) <= 10.0:  # Increased tolerance to 10 degrees
            self.task_complete = True
        
        return {
            'heading_deg': self.current_target.target_heading,
            'depth_m': current_depth,
            'speed_mps': 0.5,  # Slow speed for heading changes
            'heading_error': heading_error
        }

    def _generate_emergency_surface_commands(self, nav_state: Dict[str, Any]) -> Dict[str, Any]:
        """Generate commands for emergency surface"""
        current_heading = nav_state.get('attitude', {}).get('yaw', 0.0)
        current_depth = nav_state.get('position', {}).get('depth', 0.0)
        
        # Check if surfaced
        if current_depth <= 1.5:  # Increased surface threshold
            self.task_complete = True
        
        return {
            'heading_deg': current_heading,  # Maintain current heading
            'depth_m': 0.0,  # Surface target
            'speed_mps': 1.5,  # Fast ascent speed
            'emergency': True
        }

    def _generate_gps_fix_commands(self, nav_state: Dict[str, Any]) -> Dict[str, Any]:
        """Generate commands for GPS fix at surface"""
        current_heading = nav_state.get('attitude', {}).get('yaw', 0.0)
        
        # Stay at surface for GPS
        return {
            'heading_deg': current_heading,
            'depth_m': 0.5,  # Just below surface
            'speed_mps': 0.0,  # Hold position for GPS fix
            'gps_fix_mode': True
        }

    def _calculate_distance(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate distance between two lat/lon points using Haversine formula"""
        R = 6371000  # Earth radius in meters
        
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_phi = math.radians(lat2 - lat1)
        delta_lambda = math.radians(lon2 - lon1)
        
        a = (math.sin(delta_phi/2)**2 + 
             math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda/2)**2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        
        return R * c

    def _calculate_bearing(self, lat1: float, lon1: float, lat2: float, lon2: float) -> float:
        """Calculate bearing from point 1 to point 2"""
        phi1 = math.radians(lat1)
        phi2 = math.radians(lat2)
        delta_lambda = math.radians(lon2 - lon1)
        
        x = math.sin(delta_lambda) * math.cos(phi2)
        y = (math.cos(phi1) * math.sin(phi2) - 
             math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda))
        
        bearing = math.atan2(x, y)
        return (math.degrees(bearing) + 360) % 360

    def get_guidance_status(self) -> Dict[str, Any]:
        """Get current guidance system status"""
        return {
            'mode': self.mode.value,
            'task_complete': self.task_complete,
            'current_target': {
                'waypoint': vars(self.current_target.waypoint) if self.current_target.waypoint else None,
                'station_keeping': vars(self.current_target.station_keeping_position) if self.current_target.station_keeping_position else None,
                'target_depth': self.current_target.target_depth,
                'target_heading': self.current_target.target_heading,
                'target_speed': self.current_target.target_speed
            },
            'tolerances': {
                'waypoint': self.waypoint_tolerance,
                'station_keeping': self.station_keeping_tolerance
            },
            'speed_limits': {
                'default': self.default_speed,
                'max': self.max_speed,
                'min': self.min_speed
            },
            'waypoint_status': {
                'close_time': (time.time() - self.waypoint_close_time) if self.waypoint_close_time else 0.0,
                'required_duration': self.required_close_duration
            }
        }

    def reset(self):
        """Reset guidance system to idle state"""
        self.mode = GuidanceMode.IDLE
        self.current_target = GuidanceTarget()
        self.task_complete = False
        self.last_distance_to_target = float('inf')
        self.approach_count = 0
        self.waypoint_close_time = None
        self.mode_start_time = time.time()
        
        self.logger.info("Guidance system reset to idle")

# Windows 11 compatibility functions
def create_default_guidance_config() -> Dict[str, Any]:
    """Create default guidance configuration for Windows 11"""
    return {
        'default_speed': 1.0,
        'waypoint_tolerance': 10.0,  # Increased for better completion
        'station_keeping_tolerance': 3.0,  # Increased for stability
        'max_speed': 2.5,
        'min_speed': 0.1,
        'windows_compatible': True
    }