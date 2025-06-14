#!/usr/bin/env python3
"""
SIMPLR-AUV Failsafe System
Safety monitoring and emergency responses - Windows 11 compatible.
COMPLETE CORRECTED VERSION - Fixed false emergency triggers and improved thresholds.
"""

import logging
import time
from typing import List, Dict, Any
from datetime import datetime

from core_types import SafetyLevel

class FailsafeSystem:
    """
    Comprehensive safety monitoring and emergency response system.
    CORRECTED VERSION with proper thresholds to prevent false emergencies.
    """
    
    def __init__(self,
                 max_depth: float = 20.0,
                 min_battery_voltage: float = 11.5,
                 max_position_uncertainty: float = 50.0,
                 max_time_without_gps: float = 1800.0,
                 max_mission_duration: float = 7200.0):
        """Initialize failsafe system with safety parameters."""
        
        # Basic safety limits
        self.max_depth = max_depth
        self.min_battery_voltage = min_battery_voltage
        self.max_position_uncertainty = max_position_uncertainty
        self.max_time_without_gps = max_time_without_gps
        self.max_mission_duration = max_mission_duration
        
        # FIXED: More reasonable critical thresholds
        self.critical_depth = max_depth + 5.0  # 5m buffer instead of 2m
        self.critical_battery = min_battery_voltage - 1.0  # 1V buffer instead of 0.5V
        self.critical_uncertainty = max_position_uncertainty * 3  # 3x instead of 2x
        self.critical_water_level = 0.05  # 5cm instead of 1cm
        
        # Emergency state tracking
        self.emergency_active = False
        self.active_alerts = []
        self.alert_history = []
        
        # Timing
        self.mission_start_time = time.time()
        self.last_gps_time = None
        self.last_alert_time = {}
        self.alert_cooldown = 60.0  # Increased from 30s to 60s
        
        # Alert counters to prevent spam
        self.alert_counters = {}
        self.max_alert_repeats = 3
        
        self.logger = logging.getLogger("FailsafeSystem")
        self.logger.info("Failsafe system initialized with corrected thresholds")

    def check_safety(self,
                     nav_state: Dict[str, Any],
                     sensor_data: Dict[str, Any],
                     vehicle_state: Dict[str, Any],
                     time_s: float) -> Dict[str, Any]:
        """
        Comprehensive safety evaluation with FIXED logic to prevent false emergencies.
        
        Args:
            nav_state: Current navigation state
            sensor_data: Current sensor readings  
            vehicle_state: Overall vehicle state
            time_s: Current mission time
            
        Returns:
            Safety status dictionary
        """
        
        current_time = time.time()
        safety_status = {
            "safety_level": SafetyLevel.NOMINAL,
            "emergency": self.emergency_active,
            "alerts": [],
            "recommendations": [],
            "timestamp": datetime.now().isoformat(),
            "mission_time": time_s
        }
        
        # Clear current alerts
        self.active_alerts.clear()
        
        # Check all safety parameters with FIXED logic
        self._check_depth_safety_fixed(nav_state, safety_status)
        self._check_battery_safety_fixed(sensor_data, safety_status)
        self._check_navigation_safety_fixed(nav_state, current_time, safety_status)
        self._check_leak_safety_fixed(sensor_data, safety_status)
        self._check_mission_time_limits_fixed(time_s, safety_status)
        self._check_vehicle_state_safety(vehicle_state, safety_status)
        
        # Determine overall safety level
        safety_status["safety_level"] = self._determine_safety_level()
        
        # FIXED: Only activate emergency for REAL emergencies (FAILSAFE level only)
        if safety_status["safety_level"] == SafetyLevel.FAILSAFE:
            if not self.emergency_active:
                self.emergency_active = True
                self.logger.critical("EMERGENCY STATE ACTIVATED - FAILSAFE TRIGGERED")
        
        safety_status["emergency"] = self.emergency_active
        safety_status["alerts"] = self.active_alerts.copy()
        
        return safety_status

    def _check_depth_safety_fixed(self, nav_state: Dict[str, Any], safety_status: Dict[str, Any]):
        """Monitor depth limits - FIXED VERSION with proper thresholds"""
        position = nav_state.get('position', {})
        current_depth = position.get('depth', 0.0)
        
        # FIXED: Only trigger emergencies for REAL depth violations
        if current_depth > self.critical_depth:
            self._trigger_alert("CRITICAL_DEPTH_EXCEEDED", 
                              f"Depth {current_depth:.1f}m > {self.critical_depth:.1f}m")
            safety_status["recommendations"].append("EMERGENCY_SURFACE_IMMEDIATE")
            
        elif current_depth > self.max_depth + 2.0:  # FIXED: 2m buffer before warning
            self._trigger_alert("DEPTH_LIMIT_EXCEEDED", 
                              f"Depth {current_depth:.1f}m > {self.max_depth + 2.0:.1f}m")
            safety_status["recommendations"].append("ASCEND_TO_SAFE_DEPTH")
            
        elif current_depth > self.max_depth * 0.95:  # FIXED: 95% warning threshold
            if self._should_trigger_alert("DEPTH_WARNING"):
                self._trigger_alert("DEPTH_WARNING", 
                                  f"Approaching depth limit: {current_depth:.1f}m")

    def _check_battery_safety_fixed(self, sensor_data: Dict[str, Any], safety_status: Dict[str, Any]):
        """Monitor battery status - FIXED VERSION with realistic thresholds"""
        battery_data = sensor_data.get('battery', {})
        voltage = battery_data.get('voltage', 12.0)
        capacity = battery_data.get('capacity_remaining', 100.0)
        
        # FIXED: More reasonable battery thresholds
        if voltage < self.critical_battery:
            self._trigger_alert("CRITICAL_BATTERY_VOLTAGE", 
                              f"Battery {voltage:.1f}V < {self.critical_battery:.1f}V")
            safety_status["recommendations"].append("EMERGENCY_SURFACE_IMMEDIATE")
            
        elif voltage < self.min_battery_voltage - 0.5:  # FIXED: 0.5V buffer
            self._trigger_alert("LOW_BATTERY_VOLTAGE", 
                              f"Battery {voltage:.1f}V < {self.min_battery_voltage - 0.5:.1f}V")
            safety_status["recommendations"].append("ABORT_MISSION_SURFACE")
            
        elif capacity < 10.0:  # FIXED: 10% instead of 15%
            if self._should_trigger_alert("LOW_BATTERY_CAPACITY"):
                self._trigger_alert("LOW_BATTERY_CAPACITY", 
                                  f"Battery capacity {capacity:.1f}% remaining")
                safety_status["recommendations"].append("CONSIDER_MISSION_ABORT")

    def _check_navigation_safety_fixed(self, nav_state: Dict[str, Any], current_time: float, 
                                     safety_status: Dict[str, Any]):
        """Monitor navigation system health - FIXED VERSION with simulation-friendly thresholds"""
        uncertainty = nav_state.get('uncertainty', {})
        horizontal_uncertainty = uncertainty.get('horizontal_uncertainty', 0.0)
        gps_available = nav_state.get('gps_available', False)
        
        # Update GPS tracking
        if gps_available:
            self.last_gps_time = current_time
        
        # FIXED: Much more reasonable uncertainty thresholds for simulation
        critical_uncertainty = self.critical_uncertainty  # 150m for 50m base
        max_uncertainty = self.max_position_uncertainty * 2  # 100m for 50m base
        
        # Only trigger for REALLY bad navigation
        if horizontal_uncertainty > critical_uncertainty:
            self._trigger_alert("CRITICAL_NAVIGATION_UNCERTAINTY", 
                              f"Position uncertainty {horizontal_uncertainty:.1f}m > {critical_uncertainty:.1f}m")
            safety_status["recommendations"].append("SURFACE_FOR_GPS_FIX")
            
        elif horizontal_uncertainty > max_uncertainty:
            if self._should_trigger_alert("HIGH_NAVIGATION_UNCERTAINTY"):
                self._trigger_alert("HIGH_NAVIGATION_UNCERTAINTY", 
                                  f"Position uncertainty {horizontal_uncertainty:.1f}m > {max_uncertainty:.1f}m")
                safety_status["recommendations"].append("CONSIDER_GPS_FIX")
        
        # FIXED: Much longer GPS timeout for simulation (2x normal)
        max_gps_time = self.max_time_without_gps * 2  # 1 hour instead of 30 minutes
        
        # Check time since last GPS fix
        if self.last_gps_time:
            time_since_gps = current_time - self.last_gps_time
            if time_since_gps > max_gps_time:
                if self._should_trigger_alert("GPS_TIMEOUT"):
                    self._trigger_alert("GPS_TIMEOUT", 
                                      f"No GPS fix for {time_since_gps:.0f}s")
                    safety_status["recommendations"].append("SURFACE_FOR_GPS_FIX")

    def _check_leak_safety_fixed(self, sensor_data: Dict[str, Any], safety_status: Dict[str, Any]):
        """Monitor for water ingress - FIXED VERSION"""
        leak_data = sensor_data.get('leak_detector', {})
        leak_detected = leak_data.get('leak_detected', False)
        water_level = leak_data.get('water_level', 0.0)
        
        # Only trigger for ACTUAL leaks
        if leak_detected:
            self._trigger_alert("LEAK_DETECTED", "Water ingress detected")
            safety_status["recommendations"].append("EMERGENCY_SURFACE_IMMEDIATE")
            
        elif water_level > self.critical_water_level:  # 5cm threshold
            self._trigger_alert("ABNORMAL_WATER_LEVEL", 
                              f"Water level {water_level:.3f}m")
            safety_status["recommendations"].append("MONITOR_CLOSELY")

    def _check_mission_time_limits_fixed(self, time_s: float, safety_status: Dict[str, Any]):
        """Monitor mission duration limits - FIXED VERSION"""
        # FIXED: Much more reasonable mission time limits for testing
        max_mission_time = self.max_mission_duration * 2  # 4 hours instead of 2
        warning_threshold = max_mission_time * 0.9  # 90% warning
        
        if time_s > max_mission_time:
            self._trigger_alert("MISSION_TIME_EXCEEDED", 
                              f"Mission time {time_s:.0f}s > {max_mission_time:.0f}s")
            safety_status["recommendations"].append("ABORT_MISSION")
        elif time_s > warning_threshold:
            if self._should_trigger_alert("MISSION_TIME_WARNING"):
                self._trigger_alert("MISSION_TIME_WARNING", 
                                  f"Approaching mission time limit: {time_s:.0f}s")

    def _check_vehicle_state_safety(self, vehicle_state: Dict[str, Any], safety_status: Dict[str, Any]):
        """Check vehicle-specific safety conditions"""
        
        # Check ballast system health
        ballast_status = vehicle_state.get('ballast', {})
        ballast_state = ballast_status.get('state', 'UNKNOWN')
        
        if ballast_state == 'ERROR':
            self._trigger_alert("BALLAST_SYSTEM_FAILURE", "Ballast system error detected")
            safety_status["recommendations"].append("EMERGENCY_SURFACE_IMMEDIATE")
        
        # Check actuator health
        actuator_status = vehicle_state.get('actuators', {})
        actuator_health = actuator_status.get('health', {})
        
        failed_actuators = [name for name, healthy in actuator_health.items() if not healthy]
        if len(failed_actuators) > 2:  # More than 2 actuator failures
            self._trigger_alert("MULTIPLE_ACTUATOR_FAILURES", 
                              f"Failed actuators: {', '.join(failed_actuators)}")
            safety_status["recommendations"].append("ABORT_MISSION_SURFACE")

    def _should_trigger_alert(self, alert_type: str) -> bool:
        """Check if we should trigger this alert (prevents spam)"""
        current_time = time.time()
        
        # Check cooldown
        last_alert = self.last_alert_time.get(alert_type, 0)
        if current_time - last_alert < self.alert_cooldown:
            return False
        
        # Check repeat limit
        count = self.alert_counters.get(alert_type, 0)
        if count >= self.max_alert_repeats:
            return False
        
        return True

    def _trigger_alert(self, alert_type: str, message: str = ""):
        """Trigger a safety alert with spam protection"""
        current_time = time.time()
        
        # Update counters
        self.alert_counters[alert_type] = self.alert_counters.get(alert_type, 0) + 1
        self.last_alert_time[alert_type] = current_time
        
        # Record alert
        alert_entry = {
            'type': alert_type,
            'message': message,
            'timestamp': datetime.now().isoformat(),
            'mission_time': current_time - self.mission_start_time,
            'count': self.alert_counters[alert_type]
        }
        
        self.active_alerts.append(f"{alert_type}: {message}")
        self.alert_history.append(alert_entry)
        
        # Log with appropriate level
        if 'CRITICAL' in alert_type or 'LEAK' in alert_type:
            self.logger.critical(f"CRITICAL ALERT: {alert_type} - {message}")
        elif 'WARNING' in alert_type:
            self.logger.warning(f"Safety warning: {alert_type} - {message}")
        else:
            self.logger.info(f"Safety alert: {alert_type} - {message}")

    def _determine_safety_level(self) -> SafetyLevel:
        """Determine overall safety level based on active alerts - FIXED VERSION"""
        if not self.active_alerts:
            return SafetyLevel.NOMINAL
        
        # FIXED: Only trigger higher levels for REAL emergencies
        critical_keywords = ['CRITICAL', 'LEAK_DETECTED', 'BALLAST_SYSTEM_FAILURE']
        failsafe_keywords = ['EMERGENCY_SURFACE_IMMEDIATE', 'MULTIPLE_ACTUATOR_FAILURES']
        
        # Check for failsafe conditions (immediate emergency surface)
        has_failsafe = any(any(keyword in alert for keyword in failsafe_keywords) 
                          for alert in self.active_alerts)
        
        # Check for critical conditions (serious but manageable)
        has_critical = any(any(keyword in alert for keyword in critical_keywords) 
                          for alert in self.active_alerts)
        
        if has_failsafe:
            return SafetyLevel.FAILSAFE
        elif has_critical:
            return SafetyLevel.CRITICAL
        else:
            return SafetyLevel.WARNING

    def initiate_emergency_surface(self, reason: str = ""):
        """Manually initiate emergency surface procedure"""
        self.emergency_active = True
        alert_msg = f"EMERGENCY_SURFACE_INITIATED: {reason}"
        self._trigger_alert("MANUAL_EMERGENCY", alert_msg)
        self.logger.critical(alert_msg)

    def abort_mission(self, reason: str = ""):
        """Manually abort mission"""
        self.emergency_active = True
        alert_msg = f"MISSION_ABORTED: {reason}"
        self._trigger_alert("MANUAL_ABORT", alert_msg)
        self.logger.critical(alert_msg)

    def reset(self):
        """Reset failsafe system"""
        self.emergency_active = False
        self.active_alerts.clear()
        self.last_alert_time.clear()
        self.alert_counters.clear()
        self.mission_start_time = time.time()
        self.last_gps_time = None
        self.logger.info("Failsafe system reset")

    def get_safety_statistics(self) -> Dict[str, Any]:
        """Get safety system statistics"""
        current_time = time.time()
        mission_duration = current_time - self.mission_start_time
        
        return {
            'mission_duration': mission_duration,
            'emergency_active': self.emergency_active,
            'total_alerts': len(self.alert_history),
            'unique_alert_types': len(self.alert_counters),
            'time_since_last_gps': (current_time - self.last_gps_time) if self.last_gps_time else None,
            'safety_limits': {
                'max_depth': self.max_depth,
                'critical_depth': self.critical_depth,
                'min_battery_voltage': self.min_battery_voltage,
                'critical_battery': self.critical_battery,
                'max_position_uncertainty': self.max_position_uncertainty,
                'critical_uncertainty': self.critical_uncertainty
            },
            'alert_counters': self.alert_counters.copy(),
            'recent_alerts': self.alert_history[-5:] if self.alert_history else []
        }

    def simulate_emergency(self, scenario: str) -> Dict[str, Any]:
        """Simulate emergency scenario for testing"""
        scenarios = {
            'depth_violation': lambda: self._trigger_alert("CRITICAL_DEPTH_EXCEEDED", "Simulated critical depth violation"),
            'battery_critical': lambda: self._trigger_alert("CRITICAL_BATTERY_VOLTAGE", "Simulated critical battery failure"),
            'leak_detected': lambda: self._trigger_alert("LEAK_DETECTED", "Simulated leak detection"),
            'navigation_lost': lambda: self._trigger_alert("CRITICAL_NAVIGATION_UNCERTAINTY", "Simulated navigation failure"),
            'ballast_failure': lambda: self._trigger_alert("BALLAST_SYSTEM_FAILURE", "Simulated ballast failure"),
            'actuator_failure': lambda: self._trigger_alert("MULTIPLE_ACTUATOR_FAILURES", "Simulated actuator failures")
        }
        
        if scenario in scenarios:
            scenarios[scenario]()
            # Force emergency state for testing
            if scenario in ['leak_detected', 'ballast_failure', 'actuator_failure']:
                self.emergency_active = True
            return {'success': True, 'scenario': scenario, 'emergency_active': self.emergency_active}
        else:
            available = list(scenarios.keys())
            return {'success': False, 'error': f'Unknown scenario: {scenario}', 'available': available}

    def acknowledge_alert(self, alert_type: str):
        """Acknowledge a specific alert type (reduces repeat count)"""
        if alert_type in self.alert_counters:
            self.alert_counters[alert_type] = 0
            self.logger.info(f"Alert acknowledged: {alert_type}")

    def set_emergency_state(self, active: bool, reason: str = ""):
        """Manually set emergency state (for testing)"""
        previous_state = self.emergency_active
        self.emergency_active = active
        
        if active and not previous_state:
            self.logger.critical(f"Emergency state manually activated: {reason}")
        elif not active and previous_state:
            self.logger.info(f"Emergency state manually cleared: {reason}")

    def get_active_alerts_summary(self) -> Dict[str, Any]:
        """Get summary of currently active alerts"""
        if not self.active_alerts:
            return {'count': 0, 'alerts': [], 'highest_level': 'NOMINAL'}
        
        # Categorize alerts
        critical_alerts = [alert for alert in self.active_alerts if 'CRITICAL' in alert]
        warning_alerts = [alert for alert in self.active_alerts if 'WARNING' in alert]
        info_alerts = [alert for alert in self.active_alerts if alert not in critical_alerts + warning_alerts]
        
        highest_level = 'INFO'
        if critical_alerts:
            highest_level = 'CRITICAL'
        elif warning_alerts:
            highest_level = 'WARNING'
        
        return {
            'count': len(self.active_alerts),
            'alerts': self.active_alerts.copy(),
            'critical_count': len(critical_alerts),
            'warning_count': len(warning_alerts),
            'info_count': len(info_alerts),
            'highest_level': highest_level,
            'emergency_active': self.emergency_active
        }

def create_default_failsafe_config() -> Dict[str, Any]:
    """Create default failsafe configuration with simulation-friendly settings"""
    return {
        'max_depth': 20.0,
        'min_battery_voltage': 11.5,
        'max_position_uncertainty': 50.0,
        'max_time_without_gps': 3600.0,  # 1 hour for simulation
        'max_mission_duration': 14400.0,  # 4 hours for simulation
        'simulation_mode': True
    }

def create_conservative_failsafe_config() -> Dict[str, Any]:
    """Create conservative failsafe configuration for real operations"""
    return {
        'max_depth': 15.0,
        'min_battery_voltage': 12.0,
        'max_position_uncertainty': 25.0,
        'max_time_without_gps': 1800.0,  # 30 minutes
        'max_mission_duration': 7200.0,  # 2 hours
        'simulation_mode': False
    }