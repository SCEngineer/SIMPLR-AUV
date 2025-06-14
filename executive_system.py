#!/usr/bin/env python3
"""
SIMPLR-AUV Executive System - FIXED VERSION
Mission management and task sequencing with proper state machine implementation.
CORRECTED: Proper ballast logic, surface start, and mission flow
"""

import json
import logging
import time
from typing import List, Dict, Any, Optional
from enum import Enum
from dataclasses import dataclass

from core_types import NavigationMode, VehicleCommand

class MissionMode(Enum):
    """Mission execution modes from technical description"""
    INITIALIZE = "INITIALIZE"
    SURFACED_TRIM = "SURFACED_TRIM"
    GPS_FIX = "GPS_FIX"
    TELEMETRY = "TELEMETRY"
    GO_TO_SUBMERGED_TRIM = "GO_TO_SUBMERGED_TRIM"
    SUBMERGED_TRIM = "SUBMERGED_TRIM"
    DIVE = "DIVE"
    SWIM_TO_WAYPOINT = "SWIM_TO_WAYPOINT"
    CLIMB = "CLIMB"
    GO_TO_SURFACED_TRIM = "GO_TO_SURFACED_TRIM"
    MISSION_COMPLETE = "MISSION_COMPLETE"
    ABORT = "ABORT"
    RC_MANUAL_OVERRIDE = "RC_MANUAL_OVERRIDE"
    IDLE = "IDLE"

@dataclass
class MissionTask:
    """Individual mission task specification"""
    mode: str
    duration: Optional[float] = None
    waypoint: Optional[List[float]] = None
    target_depth: Optional[float] = None
    speed: Optional[float] = None
    parameters: Optional[Dict[str, Any]] = None

class ExecutiveSystem:
    """
    FIXED: Mission executive system with proper surface start and ballast logic
    """
    
    def __init__(self):
        """Initialize executive system"""
        self.mission_tasks: List[MissionTask] = []
        self.current_task_index = 0
        self.current_mode = MissionMode.IDLE
        self.previous_mode = MissionMode.IDLE
        
        # Timing management
        self.mission_start_time = 0.0
        self.mode_start_time = 0.0
        self.last_update_time = 0.0
        
        # Mission status
        self.mission_loaded = False
        self.mission_active = False
        self.mission_complete = False
        self.mission_aborted = False
        
        # Task completion tracking
        self.task_status = {
            'completed': False,
            'progress': 0.0,
            'error_message': '',
            'retry_count': 0
        }
        
        # Mode-specific state tracking
        self.mode_state = {
            'gps_fix_acquired': False,
            'ballast_target_reached': False,
            'depth_target_reached': False,
            'waypoint_reached': False,
            'telemetry_transmitted': False,
            'trim_achieved': False
        }
        
        # Safety and override status
        self.rc_override_active = False
        self.emergency_surface_requested = False
        self.failsafe_triggered = False
        
        self.logger = logging.getLogger("ExecutiveSystem")
        self.logger.info("FIXED Executive system initialized")

    def load_mission(self, mission_data: Dict[str, Any]) -> bool:
        """Load mission from dictionary or JSON data"""
        try:
            # Extract mission tasks
            if isinstance(mission_data, str):
                mission_dict = json.loads(mission_data)
            else:
                mission_dict = mission_data
            
            # Validate mission structure
            if 'mission' not in mission_dict:
                self.logger.error("Mission data missing 'mission' key")
                return False
            
            # Parse mission tasks
            self.mission_tasks = []
            for task_data in mission_dict['mission']:
                task = MissionTask(
                    mode=task_data.get('mode', 'IDLE'),
                    duration=task_data.get('duration'),
                    waypoint=task_data.get('waypoint'),
                    target_depth=task_data.get('target_depth'),
                    speed=task_data.get('speed', 1.0),
                    parameters=task_data.get('parameters', {})
                )
                self.mission_tasks.append(task)
            
            # Validate mission modes
            for task in self.mission_tasks:
                try:
                    MissionMode(task.mode)
                except ValueError:
                    self.logger.error(f"Invalid mission mode: {task.mode}")
                    return False
            
            self.mission_loaded = True
            self.current_task_index = 0
            self.current_mode = MissionMode.IDLE
            
            self.logger.info(f"Mission loaded successfully: {len(self.mission_tasks)} tasks")
            return True
            
        except Exception as e:
            self.logger.error(f"Failed to load mission: {e}")
            return False

    def start_mission(self) -> bool:
        """Start mission execution"""
        if not self.mission_loaded:
            self.logger.error("Cannot start mission: no mission loaded")
            return False
        
        if self.mission_active:
            self.logger.warning("Mission already active")
            return True
        
        self.mission_active = True
        self.mission_complete = False
        self.mission_aborted = False
        self.current_task_index = 0
        
        # Initialize timing
        current_time = time.time()
        self.mission_start_time = current_time
        self.mode_start_time = current_time
        self.last_update_time = current_time
        
        # Reset task status
        self.task_status = {
            'completed': False,
            'progress': 0.0,
            'error_message': '',
            'retry_count': 0
        }
        
        # Reset mode state
        self.mode_state = {key: False for key in self.mode_state}
        
        self.logger.info("FIXED Mission started at surface")
        return True

    def execute(self, nav_state: Dict[str, Any], 
               sensor_data: Dict[str, Any], 
               system_status: Dict[str, Any],
               time_s: float) -> Dict[str, Any]:
        """
        FIXED: Main executive function with proper surface start logic
        """
        current_time = time.time()
        
        # Update timing
        if self.last_update_time > 0:
            dt = current_time - self.last_update_time
        else:
            dt = 0.1
        
        # Check for RC override
        if self._check_rc_override(sensor_data):
            return self._handle_rc_override()
        
        # FIXED: Only check for REAL emergencies (FAILSAFE level only)
        if self._check_emergency_conditions(system_status):
            return self._handle_emergency()
        
        # If mission not active, return idle commands
        if not self.mission_active or self.mission_complete or self.mission_aborted:
            return self._generate_idle_commands()
        
        # Check if all tasks completed
        if self.current_task_index >= len(self.mission_tasks):
            return self._complete_mission()
        
        # Get current task
        current_task = self.mission_tasks[self.current_task_index]
        
        # Update current mode
        try:
            new_mode = MissionMode(current_task.mode)
            if new_mode != self.current_mode:
                self.previous_mode = self.current_mode
                self.current_mode = new_mode
                self.mode_start_time = current_time
                self.mode_state = {key: False for key in self.mode_state}
                self.logger.info(f"Mode transition: {self.previous_mode.value} -> {self.current_mode.value}")
        except ValueError:
            self.logger.error(f"Invalid mode in task {self.current_task_index}: {current_task.mode}")
            return self._abort_mission("Invalid mission mode")
        
        # Calculate time in current mode
        time_in_mode = current_time - self.mode_start_time
        
        # Execute current mode
        mode_commands = self._execute_current_mode(current_task, nav_state, sensor_data, time_in_mode)
        
        # Check for task completion and advance if needed
        if self.task_status.get('completed', False):
            self._advance_to_next_task()
        
        # Update timing
        self.last_update_time = current_time
        
        return mode_commands

    def _execute_current_mode(self, task: MissionTask, nav_state: Dict[str, Any], 
                             sensor_data: Dict[str, Any], time_in_mode: float) -> Dict[str, Any]:
        """Execute current mission mode with FIXED logic"""
        
        if self.current_mode == MissionMode.INITIALIZE:
            return self._execute_initialize(task, time_in_mode)
            
        elif self.current_mode == MissionMode.SURFACED_TRIM:
            return self._execute_surfaced_trim(task, nav_state, sensor_data, time_in_mode)
            
        elif self.current_mode == MissionMode.GPS_FIX:
            return self._execute_gps_fix(task, nav_state, sensor_data, time_in_mode)
            
        elif self.current_mode == MissionMode.TELEMETRY:
            return self._execute_telemetry(task, time_in_mode)
            
        elif self.current_mode == MissionMode.GO_TO_SUBMERGED_TRIM:
            return self._execute_go_to_submerged_trim(task, nav_state, sensor_data, time_in_mode)
            
        elif self.current_mode == MissionMode.SUBMERGED_TRIM:
            return self._execute_submerged_trim(task, nav_state, sensor_data, time_in_mode)
            
        elif self.current_mode == MissionMode.DIVE:
            return self._execute_dive(task, nav_state, time_in_mode)
            
        elif self.current_mode == MissionMode.SWIM_TO_WAYPOINT:
            return self._execute_swim_to_waypoint(task, nav_state, time_in_mode)
            
        elif self.current_mode == MissionMode.CLIMB:
            return self._execute_climb(task, nav_state, time_in_mode)
            
        elif self.current_mode == MissionMode.GO_TO_SURFACED_TRIM:
            return self._execute_go_to_surfaced_trim(task, nav_state, sensor_data, time_in_mode)
            
        else:
            return self._generate_idle_commands()

    def _execute_initialize(self, task: MissionTask, time_in_mode: float) -> Dict[str, Any]:
        """FIXED: Execute INITIALIZE mode - start at surface"""
        duration = task.duration or 10.0
        
        if time_in_mode >= duration:
            self.task_status['completed'] = True
            self.logger.info("Surface initialization complete")
        
        self.task_status['progress'] = min(1.0, time_in_mode / duration)
        
        return {
            'mode': 'initialize',
            'ballast_command': 'EMPTY',  # FIXED: Start with empty ballast at surface
            'guidance_mode': 'idle',
            'target_depth': 0.0,  # FIXED: Stay at surface
            'progress': self.task_status['progress']
        }

    def _execute_surfaced_trim(self, task: MissionTask, nav_state: Dict[str, Any], 
                              sensor_data: Dict[str, Any], time_in_mode: float) -> Dict[str, Any]:
        """FIXED: Execute SURFACED_TRIM mode - achieve neutral buoyancy at surface"""
        duration = task.duration or 30.0
        
        # Check ballast status from sensor data
        ballast_data = sensor_data.get('ballast', {})
        ballast_empty = ballast_data.get('is_empty', False)
        fill_level = ballast_data.get('fill_level', 1.0)
        
        # Check current depth from navigation
        current_depth = nav_state.get('position', {}).get('depth', 0.0)
        
        # FIXED: Proper completion logic - ballast empty AND at surface
        if (ballast_empty or fill_level <= 0.05) and current_depth <= 0.5:
            self.mode_state['trim_achieved'] = True
            self.task_status['completed'] = True
            self.logger.info(f"Surfaced trim achieved - ballast {fill_level*100:.1f}% full, depth {current_depth:.1f}m")
        elif time_in_mode >= duration:
            if fill_level <= 0.2 or current_depth <= 1.0:  # Close enough
                self.task_status['completed'] = True
                self.logger.info(f"Surfaced trim achieved by timeout - ballast {fill_level*100:.1f}% full")
            else:
                self.task_status['completed'] = True
                self.logger.warning("Surfaced trim timeout - may not be properly trimmed")
        
        self.task_status['progress'] = max(1.0 - fill_level, min(1.0, time_in_mode / duration))
        
        return {
            'mode': 'surfaced_trim',
            'ballast_command': 'EMPTY',  # Empty ballast for surface buoyancy
            'guidance_mode': 'idle',
            'target_depth': 0.0,  # Stay at surface
            'progress': self.task_status['progress']
        }

    def _execute_gps_fix(self, task: MissionTask, nav_state: Dict[str, Any], 
                        sensor_data: Dict[str, Any], time_in_mode: float) -> Dict[str, Any]:
        """FIXED: Execute GPS_FIX mode - only works at surface"""
        timeout = task.duration or 120.0
        
        # Check GPS status and depth
        gps_data = sensor_data.get('gps', {})
        gps_fix = gps_data.get('fix_quality', 0) > 0
        current_depth = nav_state.get('position', {}).get('depth', 0.0)
        at_surface = current_depth < 1.0
        
        if gps_fix and at_surface:
            self.mode_state['gps_fix_acquired'] = True
            self.task_status['completed'] = True
            self.logger.info("GPS fix acquired at surface")
        elif not at_surface:
            self.logger.warning(f"Cannot get GPS fix underwater (depth: {current_depth:.1f}m)")
        elif time_in_mode >= timeout:
            self.task_status['completed'] = True
            self.logger.warning("GPS fix timeout")
        
        self.task_status['progress'] = 1.0 if gps_fix else min(0.9, time_in_mode / timeout)
        
        return {
            'mode': 'gps_fix',
            'ballast_command': 'EMPTY',  # FIXED: Stay at surface for GPS
            'guidance_mode': 'gps_fix_surface',
            'target_depth': 0.0,  # FIXED: Must be at surface for GPS
            'speed': 0.0,
            'gps_fix_required': True,
            'progress': self.task_status['progress']
        }

    def _execute_telemetry(self, task: MissionTask, time_in_mode: float) -> Dict[str, Any]:
        """Execute TELEMETRY mode"""
        duration = task.duration or 30.0
        
        if time_in_mode >= duration:
            self.mode_state['telemetry_transmitted'] = True
            self.task_status['completed'] = True
            self.logger.info("Telemetry transmission complete")
        
        self.task_status['progress'] = min(1.0, time_in_mode / duration)
        
        return {
            'mode': 'telemetry',
            'ballast_command': 'HOLD',  # Hold current ballast state
            'guidance_mode': 'idle',
            'transmit_telemetry': True,
            'target_depth': 0.0,  # Stay at surface for telemetry
            'progress': self.task_status['progress']
        }

    def _execute_go_to_submerged_trim(self, task: MissionTask, nav_state: Dict[str, Any], 
                                     sensor_data: Dict[str, Any], time_in_mode: float) -> Dict[str, Any]:
        """FIXED: Execute GO_TO_SUBMERGED_TRIM mode - fill ballast tank"""
        timeout = task.duration or 60.0
        
        # Check ballast status
        ballast_data = sensor_data.get('ballast', {})
        ballast_full = ballast_data.get('is_full', False)
        fill_level = ballast_data.get('fill_level', 0.0)
        current_depth = nav_state.get('position', {}).get('depth', 0.0)
        
        # FIXED: Complete when ballast is sufficiently full OR depth increases significantly
        if ballast_full or fill_level >= 0.90 or current_depth >= 2.0:
            self.mode_state['ballast_target_reached'] = True
            self.task_status['completed'] = True
            self.logger.info(f"Submerged trim achieved - ballast {fill_level*100:.1f}% full, depth {current_depth:.1f}m")
        elif time_in_mode >= timeout:
            if fill_level >= 0.7 or current_depth >= 1.0:  # Partial success
                self.task_status['completed'] = True
                self.logger.info(f"Submerged trim achieved by timeout - ballast {fill_level*100:.1f}% full")
            else:
                self.task_status['completed'] = True
                self.logger.warning("Submerged trim timeout - ballast may not be sufficiently filled")
        
        # Progress based on ballast fill and depth
        ballast_progress = fill_level
        depth_progress = min(1.0, current_depth / 3.0)  # Target ~3m depth
        self.task_status['progress'] = max(ballast_progress, depth_progress)
        
        return {
            'mode': 'go_to_submerged_trim',
            'ballast_command': 'FILL',  # Fill ballast to increase negative buoyancy
            'guidance_mode': 'idle',
            'target_depth': max(2.0, current_depth),  # Allow sinking
            'progress': self.task_status['progress']
        }

    def _execute_submerged_trim(self, task: MissionTask, nav_state: Dict[str, Any], 
                               sensor_data: Dict[str, Any], time_in_mode: float) -> Dict[str, Any]:
        """Execute SUBMERGED_TRIM mode - maintain neutral buoyancy underwater"""
        duration = task.duration or 15.0
        current_depth = nav_state.get('position', {}).get('depth', 0.0)
        
        if time_in_mode >= duration and current_depth >= 1.0:  # Must be underwater
            self.mode_state['trim_achieved'] = True
            self.task_status['completed'] = True
            self.logger.info(f"Submerged trim stabilized at {current_depth:.1f}m depth")
        elif time_in_mode >= duration * 2:  # Extended timeout
            self.task_status['completed'] = True
            self.logger.warning("Submerged trim timeout")
        
        self.task_status['progress'] = min(1.0, time_in_mode / duration)
        
        return {
            'mode': 'submerged_trim',
            'ballast_command': 'HOLD',  # Hold ballast state for neutral buoyancy
            'guidance_mode': 'station_keeping',
            'target_depth': max(2.0, current_depth),  # Maintain underwater depth
            'progress': self.task_status['progress']
        }

    def _execute_dive(self, task: MissionTask, nav_state: Dict[str, Any], time_in_mode: float) -> Dict[str, Any]:
        """Execute DIVE mode"""
        target_depth = task.target_depth or 5.0
        current_depth = nav_state.get('position', {}).get('depth', 0.0)
        depth_tolerance = 1.0
        
        if current_depth >= (target_depth - depth_tolerance):
            self.mode_state['depth_target_reached'] = True
            self.task_status['completed'] = True
            self.logger.info(f"Dive complete: depth {current_depth:.1f}m (target: {target_depth:.1f}m)")
        elif time_in_mode >= 300.0:  # 5 minute timeout
            self.task_status['completed'] = True
            self.logger.warning(f"Dive timeout at {current_depth:.1f}m depth")
        
        depth_progress = min(1.0, current_depth / target_depth) if target_depth > 0 else 1.0
        self.task_status['progress'] = depth_progress
        
        return {
            'mode': 'dive',
            'guidance_mode': 'depth_change',
            'target_depth': target_depth,
            'speed': task.speed or 0.5,
            'ballast_command': 'HOLD',  # Maintain ballast during dive
            'progress': self.task_status['progress']
        }

    def _execute_swim_to_waypoint(self, task: MissionTask, nav_state: Dict[str, Any], 
                                 time_in_mode: float) -> Dict[str, Any]:
        """FIXED: Execute SWIM_TO_WAYPOINT mode with realistic completion"""
        if not task.waypoint or len(task.waypoint) < 2:
            self.logger.error("Invalid waypoint in SWIM_TO_WAYPOINT task")
            self.task_status['completed'] = True
            return self._generate_idle_commands()
        
        waypoint_lat = task.waypoint[0]
        waypoint_lon = task.waypoint[1]
        waypoint_depth = task.waypoint[2] if len(task.waypoint) > 2 else 5.0
        
        # Get current position
        current_pos = nav_state.get('position', {})
        current_lat = current_pos.get('latitude', 0.0)
        current_lon = current_pos.get('longitude', 0.0)
        current_depth = current_pos.get('depth', 0.0)
        
        # Calculate distance to waypoint
        lat_diff = waypoint_lat - current_lat
        lon_diff = waypoint_lon - current_lon
        horizontal_distance = ((lat_diff * 111320)**2 + (lon_diff * 111320)**2)**0.5
        depth_error = abs(current_depth - waypoint_depth)
        
        # FIXED: More realistic completion criteria
        horizontal_tolerance = 15.0  # 15 meters tolerance
        depth_tolerance = 2.0  # 2 meters depth tolerance
        
        # Check if waypoint reached
        if horizontal_distance <= horizontal_tolerance and depth_error <= depth_tolerance:
            self.mode_state['waypoint_reached'] = True
            self.task_status['completed'] = True
            self.logger.info(f"Waypoint reached: distance={horizontal_distance:.1f}m, depth_error={depth_error:.1f}m")
        elif time_in_mode >= 900.0:  # 15 minute timeout
            if horizontal_distance <= 30.0:  # Close enough
                self.task_status['completed'] = True
                self.logger.info(f"Waypoint approached within {horizontal_distance:.1f}m - close enough")
            else:
                self.task_status['completed'] = True
                self.logger.warning(f"Waypoint navigation timeout (distance={horizontal_distance:.1f}m)")
        
        # Calculate progress based on approach
        max_expected_distance = 100.0
        progress = max(0.1, 1.0 - (horizontal_distance / max_expected_distance))
        self.task_status['progress'] = min(0.95, progress)
        
        return {
            'mode': 'swim_to_waypoint',
            'guidance_mode': 'waypoint_following',
            'waypoint': [waypoint_lat, waypoint_lon],
            'target_depth': waypoint_depth,
            'speed': task.speed or 1.0,
            'ballast_command': 'HOLD',
            'progress': self.task_status['progress'],
            'distance_to_waypoint': horizontal_distance
        }

    def _execute_climb(self, task: MissionTask, nav_state: Dict[str, Any], time_in_mode: float) -> Dict[str, Any]:
        """Execute CLIMB mode"""
        target_depth = task.target_depth or 0.5
        current_depth = nav_state.get('position', {}).get('depth', 0.0)
        depth_tolerance = 1.0
        
        if current_depth <= (target_depth + depth_tolerance):
            self.mode_state['depth_target_reached'] = True
            self.task_status['completed'] = True
            self.logger.info(f"Climb complete: depth {current_depth:.1f}m (target: {target_depth:.1f}m)")
        elif time_in_mode >= 300.0:  # 5 minute timeout
            self.task_status['completed'] = True
            self.logger.warning(f"Climb timeout at {current_depth:.1f}m depth")
        
        # Progress based on depth reduction
        initial_depth = 10.0  # Assume starting depth
        if current_depth < initial_depth:
            climb_progress = (initial_depth - current_depth) / (initial_depth - target_depth)
            self.task_status['progress'] = max(0.0, min(1.0, climb_progress))
        else:
            self.task_status['progress'] = 0.0
        
        return {
            'mode': 'climb',
            'guidance_mode': 'depth_change',
            'target_depth': target_depth,
            'speed': task.speed or 0.5,
            'ballast_command': 'HOLD',
            'progress': self.task_status['progress']
        }

    def _execute_go_to_surfaced_trim(self, task: MissionTask, nav_state: Dict[str, Any], 
                                    sensor_data: Dict[str, Any], time_in_mode: float) -> Dict[str, Any]:
        """FIXED: Execute GO_TO_SURFACED_TRIM mode - empty ballast tank"""
        timeout = task.duration or 45.0
        
        # Check ballast status
        ballast_data = sensor_data.get('ballast', {})
        ballast_empty = ballast_data.get('is_empty', False)
        fill_level = ballast_data.get('fill_level', 1.0)
        current_depth = nav_state.get('position', {}).get('depth', 0.0)
        
        # FIXED: Complete when ballast is sufficiently empty OR vehicle surfaces
        if ballast_empty or fill_level <= 0.10 or current_depth <= 0.5:
            self.mode_state['ballast_target_reached'] = True
            self.task_status['completed'] = True
            self.logger.info(f"Surfaced trim achieved - ballast {fill_level*100:.1f}% full, depth {current_depth:.1f}m")
        elif time_in_mode >= timeout:
            if fill_level <= 0.3 or current_depth <= 1.5:  # Partial success
                self.task_status['completed'] = True
                self.logger.info(f"Surfaced trim achieved by timeout - ballast {fill_level*100:.1f}% full")
            else:
                self.task_status['completed'] = True
                self.logger.warning("Surfaced trim timeout - ballast may not be sufficiently emptied")
        
        # Progress based on ballast empty and depth reduction
        ballast_progress = 1.0 - fill_level
        depth_progress = max(0.0, 1.0 - (current_depth / 5.0))  # Progress as depth decreases
        self.task_status['progress'] = max(ballast_progress, depth_progress)
        
        return {
            'mode': 'go_to_surfaced_trim',
            'ballast_command': 'EMPTY',  # Empty ballast for positive buoyancy
            'guidance_mode': 'depth_change',
            'target_depth': 0.0,  # Surface target
            'progress': self.task_status['progress']
        }

    def _check_rc_override(self, sensor_data: Dict[str, Any]) -> bool:
        """Check for RC manual override"""
        # This would check RC receiver status
        # For now, return False (no override)
        return False

    def _handle_rc_override(self) -> Dict[str, Any]:
        """Handle RC manual override mode"""
        self.rc_override_active = True
        self.logger.warning("RC manual override active")
        
        return {
            'mode': 'rc_override',
            'manual_control': True,
            'guidance_mode': 'idle',
            'ballast_command': 'HOLD'
        }

    def _check_emergency_conditions(self, system_status: Dict[str, Any]) -> bool:
        """FIXED: Check for emergency conditions - only REAL emergencies"""
        # Only trigger for FAILSAFE level emergencies
        failsafe_status = system_status.get('failsafe', {})
        
        # Only emergency surface for FAILSAFE level
        if failsafe_status.get('safety_level') == 'FAILSAFE':
            return True
        
        # Check for critical alerts that require immediate action
        safety_violations = failsafe_status.get('alerts', [])
        critical_alerts = [alert for alert in safety_violations 
                          if any(keyword in alert for keyword in ['CRITICAL', 'LEAK_DETECTED', 'EMERGENCY'])]
        
        return len(critical_alerts) > 0

    def _handle_emergency(self) -> Dict[str, Any]:
        """Handle emergency conditions"""
        if not self.mission_aborted:
            self.logger.critical("Emergency condition detected - aborting mission")
            self.mission_aborted = True
        
        return {
            'mode': 'emergency_surface',
            'guidance_mode': 'emergency_surface',
            'ballast_command': 'EMPTY',  # FIXED: Empty ballast for emergency surface
            'target_depth': 0.0,
            'speed': 1.5,  # Fast ascent
            'emergency': True
        }

    def _advance_to_next_task(self):
        """Advance to next mission task"""
        self.current_task_index += 1
        self.task_status = {
            'completed': False,
            'progress': 0.0,
            'error_message': '',
            'retry_count': 0
        }
        
        if self.current_task_index < len(self.mission_tasks):
            next_task = self.mission_tasks[self.current_task_index]
            self.logger.info(f"Advancing to task {self.current_task_index + 1}: {next_task.mode}")
        else:
            self.logger.info("All mission tasks completed")

    def _complete_mission(self) -> Dict[str, Any]:
        """Complete mission execution"""
        if not self.mission_complete:
            self.mission_complete = True
            self.mission_active = False
            self.logger.info("Mission completed successfully")
        
        return {
            'mode': 'mission_complete',
            'guidance_mode': 'idle',
            'ballast_command': 'EMPTY',  # Surface at mission end
            'target_depth': 0.0,
            'mission_complete': True
        }

    def _abort_mission(self, reason: str) -> Dict[str, Any]:
        """Abort mission with specified reason"""
        if not self.mission_aborted:
            self.mission_aborted = True
            self.mission_active = False
            self.logger.error(f"Mission aborted: {reason}")
        
        return {
            'mode': 'abort',
            'guidance_mode': 'emergency_surface',
            'ballast_command': 'EMPTY',  # Emergency surface
            'target_depth': 0.0,
            'abort_reason': reason
        }

    def _generate_idle_commands(self) -> Dict[str, Any]:
        """FIXED: Generate idle commands when no mission active"""
        return {
            'mode': 'idle',
            'guidance_mode': 'idle',
            'ballast_command': 'EMPTY',  # FIXED: Default to surface buoyancy
            'target_depth': 0.0,  # FIXED: Default to surface
            'speed': 0.0
        }

    def get_mission_status(self) -> Dict[str, Any]:
        """Get current mission status"""
        return {
            'mission_loaded': self.mission_loaded,
            'mission_active': self.mission_active,
            'mission_complete': self.mission_complete,
            'mission_aborted': self.mission_aborted,
            'current_mode': self.current_mode.value,
            'current_task_index': self.current_task_index,
            'total_tasks': len(self.mission_tasks),
            'task_progress': self.task_status['progress'],
            'mission_progress': (self.current_task_index / len(self.mission_tasks)) * 100 if self.mission_tasks else 0,
            'time_in_mode': time.time() - self.mode_start_time if self.mode_start_time > 0 else 0,
            'mission_time': time.time() - self.mission_start_time if self.mission_start_time > 0 else 0,
            'rc_override_active': self.rc_override_active,
            'emergency_surface_requested': self.emergency_surface_requested,
            'surface_start': True  # Indicate this is the fixed version
        }

    def abort_mission(self, reason: str = "Manual abort"):
        """Manually abort mission"""
        self.mission_aborted = True
        self.mission_active = False
        self.logger.warning(f"Mission manually aborted: {reason}")

    def reset(self):
        """Reset executive system to initial state"""
        self.current_task_index = 0
        self.current_mode = MissionMode.IDLE
        self.mission_active = False
        self.mission_complete = False
        self.mission_aborted = False
        self.rc_override_active = False
        self.emergency_surface_requested = False
        
        # Reset timing
        self.mission_start_time = 0.0
        self.mode_start_time = 0.0
        self.last_update_time = 0.0
        
        # Reset task status
        self.task_status = {
            'completed': False,
            'progress': 0.0,
            'error_message': '',
            'retry_count': 0
        }
        
        # Reset mode state
        self.mode_state = {key: False for key in self.mode_state}
        
        self.logger.info("Executive system reset")

# FIXED: Windows 11 compatibility functions with surface start
def create_default_mission() -> Dict[str, Any]:
    """FIXED: Create default mission starting at surface"""
    return {
        "mission_info": {
            "name": "Default Surface Start Mission",
            "description": "Surface start mission with proper ballast logic",
            "version": "2.0"
        },
        "mission": [
            {"mode": "INITIALIZE", "duration": 10},  # Start at surface
            {"mode": "SURFACED_TRIM", "duration": 30},  # Achieve surface trim
            {"mode": "GPS_FIX", "duration": 60},  # Get GPS fix at surface
            {"mode": "TELEMETRY", "duration": 20},  # Send telemetry
            {"mode": "GO_TO_SUBMERGED_TRIM", "duration": 45},  # Fill ballast
            {"mode": "DIVE", "target_depth": 5.0, "duration": 120},  # Dive to 5m
            {"mode": "SWIM_TO_WAYPOINT", "waypoint": [34.0002, -117.0002, 5.0], "speed": 1.0},  # Navigate
            {"mode": "CLIMB", "target_depth": 1.0, "duration": 120},  # Climb to 1m
            {"mode": "GO_TO_SURFACED_TRIM", "duration": 45},  # Empty ballast
            {"mode": "GPS_FIX", "duration": 60},  # Get final GPS fix
            {"mode": "TELEMETRY", "duration": 20}  # Send final telemetry
        ]
    }

def load_mission_from_file(filename: str) -> Optional[Dict[str, Any]]:
    """Load mission from JSON file (Windows 11 compatible)"""
    try:
        with open(filename, 'r') as f:
            return json.load(f)
    except Exception as e:
        logging.error(f"Failed to load mission file {filename}: {e}")
        return None