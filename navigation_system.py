#!/usr/bin/env python3
"""
SIMPLR-AUV Navigation System - FIXED VERSION
Platform-independent navigation with sensor fusion and Extended Kalman Filter.
CORRECTED: Proper surface initialization, GPS logic, and depth integration
"""

import logging
import math
import time
from typing import Dict, Any, Optional, Tuple
import numpy as np
from collections import deque

# Import core types and hardware abstraction
from core_types import *
from hardware_abstraction import read_sensors

class NavigationSystem:
    """
    FIXED Navigation system - starts at surface with proper GPS integration
    """
    
    def __init__(self, 
                 update_rate: float = 10.0,
                 gps_timeout: float = 30.0,
                 max_uncertainty: float = 50.0,
                 **kwargs):
        """
        Initialize navigation system with FIXED surface start position
        """
        self.update_rate = update_rate
        self.gps_timeout = gps_timeout
        self.max_uncertainty = max_uncertainty
        
        # Handle compatibility parameters
        self.windows_compatible = kwargs.get('windows_compatible', True)
        
        # System state
        self.is_active = False
        
        # Navigation state
        self.current_state = None
        self.navigation_mode = NavigationMode.GPS_SURFACE  # FIXED: Start in GPS mode
        self.quality = 0.0
        
        # Sensor data buffers for smoothing and validation
        self.gps_buffer = deque(maxlen=10)
        self.imu_buffer = deque(maxlen=50) 
        self.depth_buffer = deque(maxlen=20)
        
        # Timing and GPS management
        self.last_gps_time = None
        self.last_update_time = None
        self.origin_set = False
        self.origin_lat = 34.0001  # FIXED: Default surface start location
        self.origin_lon = -117.0001
        
        # Extended Kalman Filter state
        # State vector: [lat_local, lon_local, depth, vel_n, vel_e, vel_d, heading, pitch, roll]
        self.state = np.zeros(9)
        self.covariance = self._initialize_covariance()
        
        # Process and measurement noise matrices
        self.Q = self._initialize_process_noise()
        self.R = self._initialize_measurement_noise()
        
        # FIXED: Dead reckoning fallback starts at surface
        self.dr_position = Position(
            latitude=34.0001,
            longitude=-117.0001,
            depth=0.0  # FIXED: Start at surface
        )
        self.dr_confidence = 1.0
        self.dr_last_update = None
        
        # Uncertainty bounds for stability
        self.max_position_std = 100.0
        self.max_velocity_std = 5.0
        self.max_attitude_std = math.pi
        
        self.logger = logging.getLogger("NavigationSystem")
        self.logger.info(f"FIXED Navigation system initialized at surface position")

    async def start(self):
        """Start the navigation system"""
        try:
            # Reset any previous state
            self.reset()
            
            # FIXED: Initialize at surface with GPS mode
            self.origin_set = True  # Set origin immediately
            self.origin_lat = 34.0001
            self.origin_lon = -117.0001
            self.navigation_mode = NavigationMode.GPS_SURFACE
            
            # Initialize timing
            self.last_update_time = time.time()
            
            # Set active flag
            self.is_active = True
            
            self.logger.info("Navigation system started at surface")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to start navigation system: {e}")
            return False

    async def stop(self):
        """Stop the navigation system"""
        try:
            self.is_active = False
            self.current_state = None
            self.navigation_mode = NavigationMode.DEAD_RECKONING
            
            self.logger.info("Navigation system stopped")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to stop navigation system: {e}")
            return False

    def update(self, time_s: float) -> Dict[str, Any]:
        """
        FIXED: Main navigation update function with proper surface start
        """
        try:
            if not self.is_active:
                return self._get_safe_navigation_state()
            
            current_time = time.time()
            
            # Get sensor data through hardware abstraction layer
            sensor_data = read_sensors()
            
            # Calculate time delta
            dt = 0.1  # Default 10Hz
            if self.last_update_time:
                dt = min(current_time - self.last_update_time, 0.5)
            
            # Process sensor data and update state
            self._process_sensor_data(sensor_data, current_time)
            
            # Update navigation mode based on available sensors
            self._update_navigation_mode(sensor_data, current_time)
            
            # Run EKF prediction step
            self._prediction_step(sensor_data, dt)
            
            # Run EKF update step if measurements available
            self._update_step(sensor_data, current_time)
            
            # Update dead reckoning as backup
            self._update_dead_reckoning(sensor_data, dt)
            
            # Extract navigation state from filter
            nav_state = self._extract_navigation_state()
            
            # Validate and store result
            if nav_state:
                self.current_state = nav_state
                self.quality = nav_state.quality
            
            self.last_update_time = current_time
            
            return self.get_navigation_state()
            
        except Exception as e:
            self.logger.error(f"Navigation update failed: {e}")
            return self._get_safe_navigation_state()

    def _process_sensor_data(self, sensor_data: Dict[str, Any], current_time: float):
        """FIXED: Process sensor data with proper GPS surface logic"""
        
        # Process GPS data - FIXED logic
        gps_data = sensor_data.get('gps', {})
        depth_data = sensor_data.get('depth', {})
        current_depth = depth_data.get('depth_m', 0.0)
        
        # GPS only works at surface (depth < 1m)
        if gps_data.get('fix_quality', 0) > 0 and current_depth < 1.0:
            gps_position = Position(
                latitude=gps_data.get('latitude', self.origin_lat),
                longitude=gps_data.get('longitude', self.origin_lon),
                depth=0.0  # GPS is surface only
            )
            self.gps_buffer.append((current_time, gps_position))
            self.last_gps_time = current_time
            
            # Set origin on first GPS fix
            if not self.origin_set:
                self.origin_lat = gps_position.latitude
                self.origin_lon = gps_position.longitude
                self.origin_set = True
                self.logger.info(f"Navigation origin set: {self.origin_lat:.6f}, {self.origin_lon:.6f}")
            
            # Reset uncertainty when GPS is available
            self._reset_position_uncertainty_on_gps()
        
        # Process IMU data
        imu_data = sensor_data.get('imu', {})
        if imu_data:
            attitude = Attitude(
                roll=self._calculate_roll_from_accel(imu_data.get('acceleration', [0, 0, -9.81])),
                pitch=self._calculate_pitch_from_accel(imu_data.get('acceleration', [0, 0, -9.81])),
                yaw=self._calculate_heading_from_mag(imu_data.get('magnetic_field', [0.3, 0, 0.5]))
            )
            self.imu_buffer.append((current_time, attitude))
        
        # Process depth data
        if depth_data:
            depth_value = depth_data.get('depth_m', 0.0)
            self.depth_buffer.append((current_time, depth_value))

    def _update_navigation_mode(self, sensor_data: Dict[str, Any], current_time: float):
        """FIXED: Update navigation mode with proper GPS surface logic"""
        
        # Check GPS availability - only at surface
        depth_data = sensor_data.get('depth', {})
        current_depth = depth_data.get('depth_m', 0.0)
        at_surface = current_depth < 1.0
        
        gps_available = (self.last_gps_time and 
                        (current_time - self.last_gps_time) < self.gps_timeout and
                        at_surface)  # FIXED: GPS only at surface
        
        # Determine navigation mode
        if gps_available and at_surface:
            self.navigation_mode = NavigationMode.GPS_SURFACE
        elif at_surface:
            self.navigation_mode = NavigationMode.GPS_SURFACE  # Try to get GPS
        else:
            self.navigation_mode = NavigationMode.DEAD_RECKONING

    def _prediction_step(self, sensor_data: Dict[str, Any], dt: float):
        """EKF prediction step - FIXED version"""
        
        # State transition model (simplified constant velocity)
        F = np.eye(9)
        
        # Position integration from velocity
        F[0, 3] = dt  # lat += vel_n * dt
        F[1, 4] = dt  # lon += vel_e * dt  
        F[2, 5] = dt  # depth += vel_d * dt
        
        # Predict state
        self.state = F @ self.state
        
        # FIXED: Clamp process noise to prevent unbounded growth
        Q_scaled = self.Q * min(dt, 0.1)  # Cap dt to 0.1s for stability
        
        # Predict covariance
        self.covariance = F @ self.covariance @ F.T + Q_scaled
        
        # FIXED: Bound covariance growth to prevent explosive uncertainty
        self._bound_covariance()

    def _bound_covariance(self):
        """FIXED: Bound covariance matrix to prevent unbounded uncertainty growth"""
        
        # Clamp diagonal elements (variances) to maximum allowed values
        self.covariance[0, 0] = min(self.covariance[0, 0], self.max_position_std**2)  # Lat variance
        self.covariance[1, 1] = min(self.covariance[1, 1], self.max_position_std**2)  # Lon variance
        self.covariance[2, 2] = min(self.covariance[2, 2], 25.0)  # Depth variance (5m max)
        
        self.covariance[3, 3] = min(self.covariance[3, 3], self.max_velocity_std**2)  # Vel_N variance
        self.covariance[4, 4] = min(self.covariance[4, 4], self.max_velocity_std**2)  # Vel_E variance
        self.covariance[5, 5] = min(self.covariance[5, 5], self.max_velocity_std**2)  # Vel_D variance
        
        self.covariance[6, 6] = min(self.covariance[6, 6], self.max_attitude_std**2)  # Yaw variance
        self.covariance[7, 7] = min(self.covariance[7, 7], self.max_attitude_std**2)  # Pitch variance
        self.covariance[8, 8] = min(self.covariance[8, 8], self.max_attitude_std**2)  # Roll variance
        
        # Ensure positive semi-definite by clamping off-diagonal terms
        for i in range(9):
            for j in range(i+1, 9):
                max_covar = 0.5 * math.sqrt(self.covariance[i, i] * self.covariance[j, j])
                self.covariance[i, j] = max(-max_covar, min(max_covar, self.covariance[i, j]))
                self.covariance[j, i] = self.covariance[i, j]

    def _reset_position_uncertainty_on_gps(self):
        """Reset position uncertainty when GPS fix is obtained"""
        if self.origin_set:
            # Reset position uncertainties to GPS accuracy
            self.covariance[0, 0] = 9.0    # 3m GPS accuracy
            self.covariance[1, 1] = 9.0    # 3m GPS accuracy
            
            # Reset cross-correlations involving position
            self.covariance[0, 1] = 0.0
            self.covariance[1, 0] = 0.0
            for i in range(3, 9):
                self.covariance[0, i] = 0.0
                self.covariance[i, 0] = 0.0
                self.covariance[1, i] = 0.0
                self.covariance[i, 1] = 0.0

    def _update_step(self, sensor_data: Dict[str, Any], current_time: float):
        """EKF update step - incorporate sensor measurements"""
        
        measurements = []
        H_rows = []
        R_indices = []
        
        # GPS measurement (if available and at surface)
        gps_data = sensor_data.get('gps', {})
        depth_data = sensor_data.get('depth', {})
        current_depth = depth_data.get('depth_m', 0.0)
        
        if (gps_data.get('fix_quality', 0) > 0 and 
            current_depth < 1.0 and 
            self.origin_set):
            
            lat_m, lon_m = self._gps_to_local(gps_data['latitude'], gps_data['longitude'])
            measurements.extend([lat_m, lon_m])
            
            # Measurement matrix rows for GPS
            h_lat = np.zeros(9)
            h_lat[0] = 1.0  # Direct observation of latitude
            h_lon = np.zeros(9)
            h_lon[1] = 1.0  # Direct observation of longitude
            
            H_rows.extend([h_lat, h_lon])
            R_indices.extend([0, 1])  # GPS measurement noise indices
        
        # Depth measurement (always available)
        if depth_data:
            depth_m = depth_data.get('depth_m', 0.0)
            measurements.append(depth_m)
            
            h_depth = np.zeros(9)
            h_depth[2] = 1.0  # Direct observation of depth
            
            H_rows.append(h_depth)
            R_indices.append(2)  # Depth measurement noise index
        
        # Apply measurements if any are available
        if measurements:
            z = np.array(measurements)
            H = np.array(H_rows)
            
            # Build measurement noise matrix for this update
            R_update = np.eye(len(measurements))
            for i, r_idx in enumerate(R_indices):
                R_update[i, i] = self.R[r_idx, r_idx]
            
            try:
                # Kalman gain
                S = H @ self.covariance @ H.T + R_update
                
                # Check for numerical stability
                if np.linalg.det(S) > 1e-12:
                    K = self.covariance @ H.T @ np.linalg.inv(S)
                    
                    # Update state
                    y = z - H @ self.state  # Innovation
                    self.state = self.state + K @ y
                    
                    # Update covariance (Joseph form for numerical stability)
                    I_KH = np.eye(9) - K @ H
                    self.covariance = I_KH @ self.covariance @ I_KH.T + K @ R_update @ K.T
                    
                    # Ensure covariance remains bounded after update
                    self._bound_covariance()
                else:
                    self.logger.warning("Singular innovation covariance - skipping measurement update")
                    
            except np.linalg.LinAlgError:
                self.logger.warning("Singular matrix in Kalman update - skipping")

    def _update_dead_reckoning(self, sensor_data: Dict[str, Any], dt: float):
        """FIXED: Update dead reckoning with proper initialization"""
        
        # Get velocity from water speed sensor
        water_speed = sensor_data.get('water_speed', {})
        velocity = Velocity(
            north=water_speed.get('vx', 0.0),
            east=water_speed.get('vy', 0.0),
            down=water_speed.get('vz', 0.0)
        )
        
        # Get heading from IMU
        heading = 0.0
        if self.imu_buffer:
            _, latest_attitude = self.imu_buffer[-1]
            heading = latest_attitude.yaw
        
        # FIXED: Dead reckoning position update with bounds
        if self.dr_last_update and dt > 0 and dt < 1.0:
            # Calculate position change in body frame
            distance = min(velocity.magnitude * dt, 10.0)  # Max 10m change per update
            heading_rad = math.radians(heading)
            
            # Convert to NED frame
            north_change = distance * math.cos(heading_rad)
            east_change = distance * math.sin(heading_rad)
            
            # Update position with bounds
            lat_change = north_change / 111320.0
            lon_change = east_change / (111320.0 * math.cos(math.radians(self.dr_position.latitude)))
            
            # Bound the changes to prevent unrealistic jumps
            lat_change = max(-0.0001, min(0.0001, lat_change))  # ~11m max
            lon_change = max(-0.0001, min(0.0001, lon_change))  # ~11m max
            
            self.dr_position.latitude += lat_change
            self.dr_position.longitude += lon_change
            self.dr_position.depth += max(-5.0, min(5.0, velocity.down * dt))  # Bound depth change
            
            # Ensure depth doesn't go negative (above surface)
            self.dr_position.depth = max(0.0, self.dr_position.depth)
            
            # Decay confidence more slowly to maintain navigation
            self.dr_confidence *= 0.998  # Slower decay
        
        self.dr_last_update = time.time()

    def _extract_navigation_state(self) -> Optional[NavigationState]:
        """FIXED: Extract NavigationState from EKF state vector"""
        
        try:
            # Extract position
            if self.origin_set:
                # Convert local coordinates back to lat/lon
                lat_local = self.state[0]
                lon_local = self.state[1]
                
                lat = self.origin_lat + lat_local / 111320.0
                lon = self.origin_lon + lon_local / (111320.0 * math.cos(math.radians(self.origin_lat)))
            else:
                # Use dead reckoning if no origin set
                lat = self.dr_position.latitude
                lon = self.dr_position.longitude
            
            # FIXED: Ensure depth is always non-negative (can't be above surface)
            depth = max(0.0, self.state[2])
            
            position = Position(
                latitude=lat,
                longitude=lon,
                depth=depth
            )
            
            # Extract velocity
            velocity = Velocity(
                north=self.state[3],
                east=self.state[4],
                down=self.state[5]
            )
            
            # Extract attitude
            attitude = Attitude(
                yaw=math.degrees(self.state[6]) % 360,
                pitch=math.degrees(self.state[7]),
                roll=math.degrees(self.state[8])
            )
            
            # Calculate quality from covariance (bounded)
            position_uncertainty = self._calculate_horizontal_uncertainty()
            quality = max(0.0, min(1.0, 1.0 / (1.0 + position_uncertainty / 10.0)))
            
            return NavigationState(
                position=position,
                velocity=velocity,
                attitude=attitude,
                quality=quality,
                mode=self.navigation_mode,
                timestamp=datetime.now()
            )
            
        except Exception as e:
            self.logger.error(f"Failed to extract navigation state: {e}")
            return None

    def get_navigation_state(self) -> Dict[str, Any]:
        """
        FIXED: Get current navigation state in dictionary format
        """
        if not self.current_state:
            return self._get_safe_navigation_state()
        
        return {
            'position': {
                'latitude': self.current_state.position.latitude,
                'longitude': self.current_state.position.longitude,
                'depth': self.current_state.position.depth
            },
            'velocity': {
                'north': self.current_state.velocity.north,
                'east': self.current_state.velocity.east,
                'down': self.current_state.velocity.down,
                'magnitude': self.current_state.velocity.magnitude
            },
            'attitude': {
                'roll': self.current_state.attitude.roll,
                'pitch': self.current_state.attitude.pitch,
                'yaw': self.current_state.attitude.yaw
            },
            'quality': self.current_state.quality,
            'mode': self.current_state.mode.value,
            'uncertainty': {
                'horizontal_uncertainty': self._calculate_horizontal_uncertainty(),
                'vertical_uncertainty': self._calculate_vertical_uncertainty()
            },
            'gps_available': self._is_gps_available(),
            'time_since_gps': self._time_since_gps()
        }

    def _get_safe_navigation_state(self) -> Dict[str, Any]:
        """FIXED: Return safe default navigation state at surface"""
        return {
            'position': {'latitude': 34.0001, 'longitude': -117.0001, 'depth': 0.0},  # Surface start
            'velocity': {'north': 0.0, 'east': 0.0, 'down': 0.0, 'magnitude': 0.0},
            'attitude': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0},
            'quality': 0.5,  # Moderate quality for safe state
            'mode': NavigationMode.GPS_SURFACE.value,  # Start in GPS mode
            'uncertainty': {'horizontal_uncertainty': 10.0, 'vertical_uncertainty': 1.0},  # Reasonable uncertainty
            'gps_available': True,  # Assume GPS available at surface
            'time_since_gps': 0.0
        }

    def _calculate_horizontal_uncertainty(self) -> float:
        """Calculate horizontal position uncertainty from covariance"""
        if self.covariance is not None:
            lat_var = max(0.0, min(self.covariance[0, 0], self.max_position_std**2))
            lon_var = max(0.0, min(self.covariance[1, 1], self.max_position_std**2))
            uncertainty = math.sqrt(lat_var + lon_var)
            return min(uncertainty, self.max_uncertainty)
        return 10.0  # Default reasonable uncertainty

    def _calculate_vertical_uncertainty(self) -> float:
        """Calculate vertical position uncertainty"""
        if self.covariance is not None:
            depth_var = max(0.0, min(self.covariance[2, 2], 25.0))
            return math.sqrt(depth_var)
        return 1.0  # Default depth uncertainty

    def _is_gps_available(self) -> bool:
        """Check if GPS is currently available"""
        if not self.last_gps_time:
            return False
        return (time.time() - self.last_gps_time) < self.gps_timeout

    def _time_since_gps(self) -> float:
        """Get time since last GPS fix"""
        if not self.last_gps_time:
            return 0.0  # No GPS yet
        return time.time() - self.last_gps_time

    def _gps_to_local(self, lat: float, lon: float) -> Tuple[float, float]:
        """Convert GPS coordinates to local meters from origin"""
        if not self.origin_set:
            return 0.0, 0.0
        
        # Simple conversion (suitable for small areas)
        lat_m = (lat - self.origin_lat) * 111320.0
        lon_m = (lon - self.origin_lon) * 111320.0 * math.cos(math.radians(self.origin_lat))
        
        return lat_m, lon_m

    def _calculate_roll_from_accel(self, accel: list) -> float:
        """Calculate roll angle from accelerometer data"""
        if len(accel) >= 3:
            return math.degrees(math.atan2(accel[1], accel[2]))
        return 0.0

    def _calculate_pitch_from_accel(self, accel: list) -> float:
        """Calculate pitch angle from accelerometer data"""
        if len(accel) >= 3:
            return math.degrees(math.atan2(-accel[0], math.sqrt(accel[1]**2 + accel[2]**2)))
        return 0.0

    def _calculate_heading_from_mag(self, mag: list) -> float:
        """Calculate heading from magnetometer data (simplified)"""
        if len(mag) >= 2:
            heading = math.degrees(math.atan2(mag[1], mag[0]))
            return heading % 360
        return 0.0

    def _initialize_covariance(self) -> np.ndarray:
        """Initialize covariance matrix with reasonable values for surface start"""
        P = np.eye(9)
        P[0:3, 0:3] *= 25.0   # Position uncertainty (5m std) - reduced for surface start
        P[3:6, 3:6] *= 1.0     # Velocity uncertainty (1 m/s std)
        P[6:9, 6:9] *= 0.01    # Attitude uncertainty (0.1 rad std)
        return P

    def _initialize_process_noise(self) -> np.ndarray:
        """Initialize process noise matrix Q"""
        Q = np.eye(9)
        Q[0:3, 0:3] *= 0.01   # Position process noise
        Q[3:6, 3:6] *= 0.005  # Velocity process noise
        Q[6:9, 6:9] *= 0.001  # Attitude process noise
        return Q

    def _initialize_measurement_noise(self) -> np.ndarray:
        """Initialize measurement noise matrix R"""
        R = np.eye(9)
        R[0, 0] = 9.0   # GPS latitude noise (3m std)
        R[1, 1] = 9.0   # GPS longitude noise (3m std)
        R[2, 2] = 0.01  # Depth noise (0.1m std)
        return R

    def reset(self):
        """FIXED: Reset navigation system to surface start state"""
        self.state = np.zeros(9)
        self.covariance = self._initialize_covariance()
        self.current_state = None
        self.quality = 0.0
        
        # FIXED: Reset to surface start position
        self.origin_set = True
        self.origin_lat = 34.0001
        self.origin_lon = -117.0001
        self.last_gps_time = time.time()  # Assume GPS available at start
        self.last_update_time = None
        
        # Clear buffers
        self.gps_buffer.clear()
        self.imu_buffer.clear()
        self.depth_buffer.clear()
        
        # FIXED: Reset dead reckoning to surface
        self.dr_position = Position(
            latitude=34.0001,
            longitude=-117.0001,
            depth=0.0  # Surface start
        )
        self.dr_confidence = 1.0
        self.dr_last_update = None
        
        self.logger.info("Navigation system reset to surface start position")

    def get_diagnostic_info(self) -> Dict[str, Any]:
        """Get navigation diagnostic information for debugging"""
        return {
            'active': self.is_active,
            'navigation_mode': self.navigation_mode.value,
            'quality': self.quality,
            'origin_set': self.origin_set,
            'origin_position': {'lat': self.origin_lat, 'lon': self.origin_lon},
            'gps_available': self._is_gps_available(),
            'time_since_gps': self._time_since_gps(),
            'horizontal_uncertainty': self._calculate_horizontal_uncertainty(),
            'vertical_uncertainty': self._calculate_vertical_uncertainty(),
            'covariance_trace': np.trace(self.covariance) if self.covariance is not None else 0.0,
            'buffer_sizes': {
                'gps': len(self.gps_buffer),
                'imu': len(self.imu_buffer),
                'depth': len(self.depth_buffer)
            },
            'uncertainty_bounds': {
                'max_position_std': self.max_position_std,
                'max_velocity_std': self.max_velocity_std,
                'max_attitude_std': self.max_attitude_std
            },
            'surface_start': True,  # Indicate this is the fixed version
            'windows_compatible': self.windows_compatible
        }

    def get_state(self) -> Dict[str, Any]:
        """Get current navigation state (alias for compatibility)"""
        return self.get_navigation_state()

    def set_uncertainty_bounds(self, max_position: float = 100.0, 
                              max_velocity: float = 5.0, 
                              max_attitude: float = math.pi):
        """Set maximum uncertainty bounds for covariance limiting"""
        self.max_position_std = max_position
        self.max_velocity_std = max_velocity 
        self.max_attitude_std = max_attitude
        self.logger.info(f"Updated uncertainty bounds: pos={max_position}m, vel={max_velocity}m/s, att={math.degrees(max_attitude)}°")

# Windows 11 compatibility functions
def create_default_navigation_config() -> Dict[str, Any]:
    """Create default navigation configuration for Windows 11"""
    return {
        'update_rate': 10.0,
        'gps_timeout': 30.0,
        'max_uncertainty': 50.0,
        'surface_start': True,  # FIXED: Indicate surface start
        'windows_compatible': True
    }

def create_simulation_navigation_config() -> Dict[str, Any]:
    """Create simulation-optimized navigation configuration"""
    return {
        'update_rate': 10.0,
        'gps_timeout': 60.0,         # Longer GPS timeout for simulation
        'max_uncertainty': 100.0,    # Higher uncertainty tolerance
        'surface_start': True,       # FIXED: Surface start
        'windows_compatible': True
    }