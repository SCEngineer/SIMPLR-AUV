#!/usr/bin/env python3
"""
SIMPLR-AUV Ballast Tank Control Module
Manages ballast tank fill/empty cycles, buoyancy control, and state transitions.
Corrected for consistency with technical description and proper state management.
"""

import logging
import time
from typing import Dict, Any
from core_types import BallastState

class BallastTank:
    """
    Ballast tank control system with accurate mass properties from technical description.
    Provides ±4.37 lb buoyancy change with 0.070 ft³ void volume.
    """
    
    def __init__(self,
                 void_volume: float = 0.070,  # ft³ from tech description
                 max_buoyancy_change: float = 4.37,  # lb from tech description
                 fill_rate_lps: float = 2.0,  # 2 GPM converted to L/s
                 empty_rate_lps: float = 2.5):  # Faster empty due to pressure
        """
        Initialize the ballast tank with specifications from technical description.
        
        Args:
            void_volume: Tank void volume in ft³ (0.070 ft³)
            max_buoyancy_change: Maximum buoyancy change in pounds (±4.37 lb)
            fill_rate_lps: Fill rate in liters per second
            empty_rate_lps: Empty rate in liters per second
        """
        self.state = BallastState.EMPTY
        self.fill_level = 0.0  # 0.0 = empty, 1.0 = full
        self.target_state = BallastState.EMPTY
        
        # Component health status
        self.pump_operational = True
        self.valve_operational = True
        
        # Physical parameters from technical description
        self.void_volume = void_volume  # ft³
        self.max_buoyancy_change = max_buoyancy_change  # lb
        self.tank_weight = 5.83  # lb from tech description
        
        # Operational parameters
        self.fill_rate = fill_rate_lps / 1000.0  # Convert to m³/s
        self.empty_rate = empty_rate_lps / 1000.0  # Convert to m³/s
        
        # Convert void volume to metric
        self.void_volume_m3 = void_volume * 0.0283168  # ft³ to m³
        
        # State timing
        self.state_start_time = time.time()
        self.last_update_time = time.time()
        
        # Emergency blow parameters
        self.emergency_blow_active = False
        self.emergency_blow_rate = 5.0 / 1000.0  # 5 L/s emergency blow rate
        
        self.logger = logging.getLogger("BallastTank")
        self.logger.info(f"Ballast tank initialized: {void_volume:.3f} ft³, ±{max_buoyancy_change:.1f} lb")

    def update(self, command: str, dt: float) -> Dict[str, Any]:
        """
        Update ballast system based on command and time delta.
        
        Args:
            command: One of ["FILL", "EMPTY", "HOLD", "EMERGENCY_BLOW"]
            dt: Time step in seconds
            
        Returns:
            Dictionary with ballast status and buoyancy information
        """
        current_time = time.time()
        
        # Handle emergency blow with highest priority
        if command == "EMERGENCY_BLOW" or self.emergency_blow_active:
            return self._handle_emergency_blow(dt)
        
        # Check system health
        if not (self.pump_operational and self.valve_operational):
            self.state = BallastState.ERROR
            self.logger.error("Ballast system failure detected")
            return self.get_ballast_status()
        
        # Process normal commands
        previous_state = self.state
        
        if command == "FILL":
            self._process_fill_command(dt)
        elif command == "EMPTY":
            self._process_empty_command(dt)
        elif command == "HOLD":
            self._process_hold_command()
        else:
            self.logger.warning(f"Invalid ballast command: {command}")
        
        # Log state changes
        if self.state != previous_state:
            self.logger.info(f"Ballast state changed: {previous_state.name} -> {self.state.name}")
            self.state_start_time = current_time
        
        self.last_update_time = current_time
        return self.get_ballast_status()

    def _process_fill_command(self, dt: float):
        """Process FILL command"""
        if self.state in [BallastState.EMPTY, BallastState.EMPTYING]:
            self.state = BallastState.FILLING
        
        if self.state == BallastState.FILLING:
            # Calculate fill rate based on void volume
            volume_rate = self.fill_rate / self.void_volume_m3  # Normalized rate
            self.fill_level += volume_rate * dt
            
            if self.fill_level >= 1.0:
                self.fill_level = 1.0
                self.state = BallastState.FULL
                self.logger.info("Ballast tank full")

    def _process_empty_command(self, dt: float):
        """Process EMPTY command"""
        if self.state in [BallastState.FULL, BallastState.FILLING]:
            self.state = BallastState.EMPTYING
        
        if self.state == BallastState.EMPTYING:
            # Calculate empty rate based on void volume
            volume_rate = self.empty_rate / self.void_volume_m3  # Normalized rate
            self.fill_level -= volume_rate * dt
            
            if self.fill_level <= 0.0:
                self.fill_level = 0.0
                self.state = BallastState.EMPTY
                self.logger.info("Ballast tank empty")

    def _process_hold_command(self):
        """Process HOLD command - maintain current state"""
        # Determine steady state based on current fill level
        if self.fill_level >= 0.99:
            self.state = BallastState.FULL
        elif self.fill_level <= 0.01:
            self.state = BallastState.EMPTY
        # Otherwise maintain current transitional state

    def _handle_emergency_blow(self, dt: float) -> Dict[str, Any]:
        """Handle emergency blow procedure"""
        if not self.emergency_blow_active:
            self.emergency_blow_active = True
            self.logger.critical("EMERGENCY BLOW ACTIVATED")
        
        self.state = BallastState.EMPTYING
        
        # Emergency blow at maximum rate
        volume_rate = self.emergency_blow_rate / self.void_volume_m3
        self.fill_level -= volume_rate * dt
        
        if self.fill_level <= 0.0:
            self.fill_level = 0.0
            self.state = BallastState.EMPTY
            self.emergency_blow_active = False
            self.logger.info("Emergency blow complete")
        
        return self.get_ballast_status()

    def emergency_blow(self):
        """Initiate emergency blow procedure"""
        self.emergency_blow_active = True
        self.logger.critical("Emergency blow initiated")

    def get_ballast_status(self) -> Dict[str, Any]:
        """
        Return comprehensive ballast system status.
        
        Returns:
            Dictionary with current state, fill level, buoyancy force, and diagnostics
        """
        buoyancy_force = self._calculate_buoyancy_force()
        
        # Calculate state timing
        time_in_state = time.time() - self.state_start_time
        
        # Estimate time to completion for transitional states
        time_to_complete = None
        if self.state == BallastState.FILLING:
            remaining_volume = (1.0 - self.fill_level) * self.void_volume_m3
            time_to_complete = remaining_volume / self.fill_rate if self.fill_rate > 0 else 0
        elif self.state == BallastState.EMPTYING:
            remaining_volume = self.fill_level * self.void_volume_m3
            time_to_complete = remaining_volume / self.empty_rate if self.empty_rate > 0 else 0
        
        return {
            "state": self.state.name,
            "fill_level": round(self.fill_level, 3),
            "buoyancy_force_lb": round(buoyancy_force, 2),
            "is_full": self.state == BallastState.FULL,
            "is_empty": self.state == BallastState.EMPTY,
            "is_transitioning": self.state in [BallastState.FILLING, BallastState.EMPTYING],
            "pump_operational": self.pump_operational,
            "valve_operational": self.valve_operational,
            "emergency_blow_active": self.emergency_blow_active,
            "time_in_state": round(time_in_state, 1),
            "time_to_complete": round(time_to_complete, 1) if time_to_complete else None,
            "void_volume_ft3": self.void_volume,
            "tank_weight_lb": self.tank_weight
        }

    def _calculate_buoyancy_force(self) -> float:
        """
        Calculate buoyancy contribution based on fill level and technical specifications.
        
        When empty: Tank provides +4.37 lb positive buoyancy (void space filled with air)
        When full: Tank provides approximately neutral buoyancy (void space filled with water)
        
        Returns:
            Buoyancy force in pounds
        """
        # From technical description:
        # - Empty tank: +4.37 lb positive buoyancy from void space
        # - Full tank: ~0 lb (water-filled void provides no buoyancy)
        
        # Linear interpolation between empty and full states
        empty_buoyancy = self.max_buoyancy_change  # +4.37 lb when empty
        full_buoyancy = 0.0  # 0 lb when full
        
        buoyancy = empty_buoyancy * (1.0 - self.fill_level) + full_buoyancy * self.fill_level
        
        return buoyancy

    def get_trim_contribution(self) -> Dict[str, float]:
        """
        Get ballast tank contribution to vehicle trim.
        
        Returns:
            Dictionary with trim analysis
        """
        buoyancy_lb = self._calculate_buoyancy_force()
        
        # Vehicle mass properties from technical description
        vehicle_weight_lb = 75.52
        surfaced_target_buoyancy = 0.0  # Neutral surfaced
        submerged_target_buoyancy = 0.05  # Slightly positive submerged
        
        return {
            "ballast_buoyancy_lb": buoyancy_lb,
            "vehicle_weight_lb": vehicle_weight_lb,
            "net_vehicle_buoyancy_surfaced": buoyancy_lb - vehicle_weight_lb + 76.10,
            "net_vehicle_buoyancy_submerged": buoyancy_lb - vehicle_weight_lb + 78.43,
            "surfaced_trim_achieved": abs(buoyancy_lb + 76.10 - vehicle_weight_lb) < 0.1,
            "submerged_trim_achieved": abs(buoyancy_lb + 78.43 - vehicle_weight_lb - 0.05) < 0.1
        }

    def set_component_health(self, pump_ok: bool, valve_ok: bool):
        """Set component health status for fault simulation"""
        self.pump_operational = pump_ok
        self.valve_operational = valve_ok
        
        if not (pump_ok and valve_ok):
            self.state = BallastState.ERROR
            self.logger.error(f"Component failure: pump={pump_ok}, valve={valve_ok}")

    def reset(self):
        """Reset ballast system to initial state"""
        self.state = BallastState.EMPTY
        self.fill_level = 0.0
        self.target_state = BallastState.EMPTY
        self.pump_operational = True
        self.valve_operational = True
        self.emergency_blow_active = False
        self.state_start_time = time.time()
        self.logger.info("Ballast tank reset to empty state")

    def get_estimated_state_times(self) -> Dict[str, float]:
        """
        Get estimated times for state transitions.
        
        Returns:
            Dictionary with time estimates in seconds
        """
        empty_to_full_time = self.void_volume_m3 / self.fill_rate
        full_to_empty_time = self.void_volume_m3 / self.empty_rate
        emergency_blow_time = self.void_volume_m3 / self.emergency_blow_rate
        
        return {
            "empty_to_full_seconds": round(empty_to_full_time, 1),
            "full_to_empty_seconds": round(full_to_empty_time, 1),
            "emergency_blow_seconds": round(emergency_blow_time, 1)
        }

    def __str__(self) -> str:
        """String representation of ballast tank state"""
        status = self.get_ballast_status()
        return (f"BallastTank(state={status['state']}, "
                f"fill={status['fill_level']:.1%}, "
                f"buoyancy={status['buoyancy_force_lb']:.1f}lb)")
