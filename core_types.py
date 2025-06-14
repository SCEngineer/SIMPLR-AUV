#!/usr/bin/env python3
"""
SIMPLR-AUV Core Types Module
Defines all data structures, enums, and utility functions used throughout the system.
Corrected for consistency with technical description and proper typing.
"""

from dataclasses import dataclass, field
from typing import Optional, Protocol, Dict, Any, List
from enum import Enum, auto
from datetime import datetime
import math

# ------------------- ENUMS -------------------

class NavigationMode(Enum):
    IDLE = "IDLE"
    GPS_FIX = "GPS_FIX"
    GPS_SURFACE = "GPS_SURFACE"
    SUBMERGE = "SUBMERGE"
    SWIM_TO_WAYPOINT = "SWIM_TO_WAYPOINT"
    CLIMB = "CLIMB"
    DIVE = "DIVE"
    SURFACED_TRIM = "SURFACED_TRIM"
    SUBMERGED_TRIM = "SUBMERGED_TRIM"
    TELEMETRY = "TELEMETRY"
    GO_TO_SURFACED_TRIM = "GO_TO_SURFACED_TRIM"
    GO_TO_SUBMERGED_TRIM = "GO_TO_SUBMERGED_TRIM"
    RC_OVERRIDE = "RC_OVERRIDE"
    DEAD_RECKONING = "DEAD_RECKONING"
    ACOUSTIC_POSITIONING = "ACOUSTIC_POSITIONING"
    VISUAL_SLAM = "VISUAL_SLAM"

class EnergyMode(Enum):
    NORMAL = "NORMAL"
    LOW_POWER = "LOW_POWER"
    CRITICAL = "CRITICAL"

class SafetyLevel(Enum):
    NOMINAL = "NOMINAL"
    WARNING = "WARNING"
    CRITICAL = "CRITICAL"
    FAILSAFE = "FAILSAFE"

class SensorType(Enum):
    IMU = "IMU"
    COMPASS = "COMPASS"
    DEPTH = "DEPTH"
    GPS = "GPS"
    DVL = "DVL"
    PADDLEWHEEL = "PADDLEWHEEL"
    WATER_SPEED = "WATER_SPEED"
    BATTERY = "BATTERY"
    LEAK_DETECTOR = "LEAK_DETECTOR"

class ThrusterID(Enum):
    MAIN = "MAIN"
    AUXILIARY = "AUXILIARY"
    BOW_HORIZONTAL = "BOW_HORIZONTAL"
    STERN_HORIZONTAL = "STERN_HORIZONTAL"
    BOW_VERTICAL = "BOW_VERTICAL"
    STERN_VERTICAL = "STERN_VERTICAL"
    PORT_LATERAL = "PORT_LATERAL"
    STARBOARD_LATERAL = "STARBOARD_LATERAL"

class VehicleState(Enum):
    IDLE = "IDLE"
    ACTIVE = "ACTIVE"
    EMERGENCY = "EMERGENCY"
    FAULT = "FAULT"

class LogLevel(Enum):
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    DEBUG = "DEBUG"

class BallastState(Enum):
    EMPTY = auto()
    FILLING = auto()
    FULL = auto()
    EMPTYING = auto()
    ERROR = auto()

class GuidanceMode(Enum):
    IDLE = "idle"
    WAYPOINT_FOLLOWING = "waypoint_following"
    STATION_KEEPING = "station_keeping"
    DEPTH_CHANGE = "depth_change"
    HEADING_HOLD = "heading_hold"
    EMERGENCY_SURFACE = "emergency_surface"
    GPS_FIX_SURFACE = "gps_fix_surface"

class ControlMode(Enum):
    MANUAL = "MANUAL"
    AUTOMATIC = "AUTOMATIC"
    EMERGENCY = "EMERGENCY"
    SAFE = "SAFE"

# ------------------- DATA CLASSES -------------------

@dataclass
class Position:
    latitude: float = 0.0
    longitude: float = 0.0
    depth: float = 0.0
    
    # Legacy compatibility for x, y, z coordinates
    @property
    def x(self) -> float:
        return self.latitude
    
    @property
    def y(self) -> float:
        return self.longitude
    
    @property
    def z(self) -> float:
        return self.depth

@dataclass
class Velocity:
    north: float = 0.0
    east: float = 0.0
    down: float = 0.0
    
    # Legacy compatibility
    @property
    def vx(self) -> float:
        return self.north
    
    @property
    def vy(self) -> float:
        return self.east
    
    @property
    def vz(self) -> float:
        return self.down
    
    @property
    def magnitude(self) -> float:
        return math.sqrt(self.north**2 + self.east**2 + self.down**2)
    
    @property
    def forward(self) -> float:
        return self.north
    
    @property
    def heading(self) -> float:
        return math.degrees(math.atan2(self.east, self.north)) % 360

@dataclass
class Attitude:
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    
    # Legacy compatibility
    @property
    def heading(self) -> float:
        return self.yaw

@dataclass
class NavigationState:
    position: Position = field(default_factory=Position)
    velocity: Velocity = field(default_factory=Velocity)
    attitude: Attitude = field(default_factory=Attitude)
    quality: float = 0.0
    mode: NavigationMode = NavigationMode.DEAD_RECKONING
    timestamp: datetime = field(default_factory=datetime.now)
    
    # Legacy compatibility
    @property
    def x(self) -> float:
        return self.position.latitude
    
    @property
    def y(self) -> float:
        return self.position.longitude
    
    @property
    def depth(self) -> float:
        return self.position.depth
    
    @property
    def heading(self) -> float:
        return self.attitude.yaw
    
    @property
    def speed(self) -> float:
        return self.velocity.magnitude

@dataclass
class VehicleCommand:
    surge: float = 0.0
    sway: float = 0.0
    heave: float = 0.0
    yaw_rate: float = 0.0
    
    # Legacy compatibility
    @property
    def thrust(self) -> float:
        return self.surge
    
    @property
    def rudder_deg(self) -> float:
        return self.yaw_rate * 30.0  # Convert to degrees
    
    @property
    def elevator_deg(self) -> float:
        return self.heave * 30.0  # Convert to degrees
    
    def to_dict(self) -> Dict[str, float]:
        return {
            'surge': self.surge,
            'sway': self.sway,
            'heave': self.heave,
            'yaw_rate': self.yaw_rate,
            'thrust': self.thrust,
            'rudder_deg': self.rudder_deg,
            'elevator_deg': self.elevator_deg
        }

@dataclass
class ThrusterCommand:
    thruster_id: ThrusterID
    thrust: float
    timestamp: datetime = field(default_factory=datetime.now)
    
    # Legacy compatibility
    @property
    def id(self) -> ThrusterID:
        return self.thruster_id
    
    @property
    def power(self) -> float:
        return self.thrust

@dataclass
class ActuatorCommands:
    rudder_angle: float = 0.0
    elevator_angle: float = 0.0
    thrust: float = 0.0

@dataclass
class SensorReading:
    sensor_type: SensorType
    value: Any
    timestamp: datetime = field(default_factory=datetime.now)
    quality: float = 1.0

@dataclass
class GPSData:
    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    speed: float = 0.0
    fix_quality: int = 0
    satellites: int = 0
    timestamp: float = 0.0
    
    # Legacy compatibility
    @property
    def lat(self) -> float:
        return self.latitude
    
    @property
    def lon(self) -> float:
        return self.longitude

@dataclass
class CompassData:
    heading_deg: float = 0.0

@dataclass
class DepthReading:
    depth: float = 0.0
    pressure_pa: float = 0.0
    temperature: float = 0.0

@dataclass
class IMUData:
    acceleration: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    angular_velocity: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    magnetic_field: List[float] = field(default_factory=lambda: [0.0, 0.0, 0.0])
    temperature: float = 0.0
    
    # Legacy compatibility
    @property
    def accel(self) -> List[float]:
        return self.acceleration
    
    @property
    def gyro(self) -> List[float]:
        return self.angular_velocity
    
    @property
    def roll(self) -> float:
        return math.degrees(math.atan2(self.acceleration[1], self.acceleration[2]))
    
    @property
    def pitch(self) -> float:
        return math.degrees(math.atan2(-self.acceleration[0], 
                                     math.sqrt(self.acceleration[1]**2 + self.acceleration[2]**2)))

@dataclass
class WaterSpeedData:
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0

@dataclass
class BatteryData:
    voltage: float = 12.0
    current: float = 0.0
    capacity_remaining: float = 100.0
    power: float = 0.0

@dataclass
class LeakDetectorData:
    leak_detected: bool = False
    water_level: float = 0.0

@dataclass
class SystemData:
    timestamp: float = 0.0
    health_status: str = "HEALTHY"
    diagnostic_code: int = 0
    error_message: str = ""

@dataclass
class MissionState:
    current_task_index: int = 0
    completed: bool = False

@dataclass
class EnergyStatus:
    mode: EnergyMode = EnergyMode.NORMAL
    battery_level: float = 100.0

@dataclass
class SystemStatus:
    navigation: NavigationState = field(default_factory=NavigationState)
    energy: EnergyStatus = field(default_factory=EnergyStatus)
    safety: SafetyLevel = SafetyLevel.NOMINAL

@dataclass
class ConfigParameters:
    max_depth: float = 20.0
    max_speed: float = 2.0

@dataclass
class SystemConfiguration:
    params: ConfigParameters = field(default_factory=ConfigParameters)

@dataclass
class Waypoint:
    latitude: float
    longitude: float
    depth: float
    id: str = ""
    name: str = ""

# ------------------- INTERFACES -------------------

class ITelemetrySystem(Protocol):
    def log(self, time_s: float, est_state: Dict[str, Any], 
           true_state: Dict[str, Any], mode: str, gps_fix: bool) -> None: ...
    def close(self) -> None: ...

# ------------------- CONSTANTS & UTILS -------------------

EARTH_RADIUS = 6371000  # meters

def calculate_distance(pos1: Position, pos2: Position) -> float:
    """Calculate distance between two positions using Haversine formula"""
    lat1, lon1 = math.radians(pos1.latitude), math.radians(pos1.longitude)
    lat2, lon2 = math.radians(pos2.latitude), math.radians(pos2.longitude)
    
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    
    a = (math.sin(dlat/2)**2 + 
         math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2)
    c = 2 * math.asin(math.sqrt(a))
    
    return EARTH_RADIUS * c

def calculate_bearing(pos1: Position, pos2: Position) -> float:
    """Calculate bearing from pos1 to pos2"""
    lat1, lon1 = math.radians(pos1.latitude), math.radians(pos1.longitude)
    lat2, lon2 = math.radians(pos2.latitude), math.radians(pos2.longitude)
    
    dlon = lon2 - lon1
    
    y = math.sin(dlon) * math.cos(lat2)
    x = (math.cos(lat1) * math.sin(lat2) - 
         math.sin(lat1) * math.cos(lat2) * math.cos(dlon))
    
    bearing = math.atan2(y, x)
    return (math.degrees(bearing) + 360) % 360

def degrees_to_radians(deg: float) -> float:
    return deg * math.pi / 180.0

def radians_to_degrees(rad: float) -> float:
    return rad * 180.0 / math.pi

# Legacy compatibility functions
def calculate_distance_legacy(x1: float, y1: float, x2: float, y2: float) -> float:
    """Legacy distance calculation for backward compatibility"""
    return math.hypot(x2 - x1, y2 - y1)

def calculate_bearing_legacy(x1: float, y1: float, x2: float, y2: float) -> float:
    """Legacy bearing calculation for backward compatibility"""
    return math.degrees(math.atan2(y2 - y1, x2 - x1)) % 360
