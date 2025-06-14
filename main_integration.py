#!/usr/bin/env python3
"""
SIMPLR-AUV Main Integration Module - WORKING VERSION
Clean working version for Windows 11 with proper surface start and ballast integration.
"""

import asyncio
import logging
import time
import json
import sys
import os
import threading
import webbrowser
import signal
from pathlib import Path
from typing import Dict, Any, Optional
import traceback

# WebSocket imports
try:
    import websockets
    WEBSOCKETS_AVAILABLE = True
except ImportError:
    WEBSOCKETS_AVAILABLE = False
    print("[WARNING] websockets package not installed. GUI features disabled.")

# HTTP server imports
from http.server import HTTPServer, SimpleHTTPRequestHandler

# Global variables for AUV modules
AUV_MODULES_IMPORTED = False
read_sensors = None
send_actuator_commands = None
initialize_hardware = None
set_simulation_mode = None
get_hardware_status = None
NavigationSystem = None
GuidanceSystem = None
ControlSystem = None
create_default_control_config = None
BallastTank = None
ActuatorModule = None
ExecutiveSystem = None
create_default_mission = None
load_mission_from_file = None
FailsafeSystem = None

def import_auv_modules():
    """Import AUV modules with proper error handling"""
    global AUV_MODULES_IMPORTED
    global read_sensors, send_actuator_commands, initialize_hardware, set_simulation_mode, get_hardware_status
    global NavigationSystem, GuidanceSystem, ControlSystem, create_default_control_config
    global BallastTank, ActuatorModule, ExecutiveSystem, create_default_mission, load_mission_from_file
    global FailsafeSystem
    
    if AUV_MODULES_IMPORTED:
        return True
    
    try:
        import core_types
        from hardware_abstraction import read_sensors, send_actuator_commands, initialize_hardware, set_simulation_mode, get_hardware_status
        from navigation_system import NavigationSystem
        from guidance_system import GuidanceSystem
        from control_system import ControlSystem, create_default_control_config
        from ballast_tank import BallastTank
        from actuators import ActuatorModule
        from executive_system import ExecutiveSystem, create_default_mission, load_mission_from_file
        from failsafe_system import FailsafeSystem
        
        AUV_MODULES_IMPORTED = True
        print("[OK] All AUV modules imported successfully")
        return True
    except ImportError as e:
        print(f"[ERROR] Import error: {e}")
        return False

class AUVWebSocketServer:
    """WebSocket server for real-time AUV data streaming"""
    
    def __init__(self, port=8765):
        self.port = port
        self.clients = set()
        self.current_data = {}
        self.server = None
        
    async def register_client(self, websocket, path=None):
        """Register new client connection"""
        self.clients.add(websocket)
        print(f"[GUI] Client connected from {websocket.remote_address}. Total clients: {len(self.clients)}")
        
        try:
            if self.current_data:
                await websocket.send(json.dumps(self.current_data))
                print(f"[GUI] Sent initial data to client")
            
            async for message in websocket:
                print(f"[GUI] Received from client: {message}")
                await websocket.send(json.dumps({"status": "received"}))
                
        except websockets.exceptions.ConnectionClosed:
            print(f"[GUI] Client disconnected normally")
        except Exception as e:
            print(f"[GUI] Client error: {e}")
        finally:
            self.clients.discard(websocket)
            print(f"[GUI] Client removed. Total: {len(self.clients)}")
    
    async def broadcast_data(self, data):
        """Broadcast data to all connected clients"""
        if not self.clients:
            return
            
        self.current_data = data
        message = json.dumps(data)
        disconnected = []
        
        for client in self.clients:
            try:
                await client.send(message)
            except Exception as e:
                disconnected.append(client)
        
        for client in disconnected:
            self.clients.discard(client)
    
    async def start_server(self):
        """Start the WebSocket server"""
        if not WEBSOCKETS_AVAILABLE:
            print("[WARNING] WebSocket server cannot start - websockets package not installed")
            return None
        
        try:
            self.server = await websockets.serve(self.register_client, "localhost", self.port)
            print(f"[GUI] WebSocket server started on ws://localhost:{self.port}")
            return self.server
        except Exception as e:
            print(f"[GUI] Failed to start WebSocket server: {e}")
            return None

class AUVGUIManager:
    """Manages the AUV GUI integration"""
    
    def __init__(self, auv_system):
        self.auv_system = auv_system
        self.websocket_server = None
        if WEBSOCKETS_AVAILABLE:
            self.websocket_server = AUVWebSocketServer()
        self.http_server = None
        self.gui_thread = None
        self.mission_start_time = None
        
    def create_gui_html(self):
        """Create the GUI HTML file content"""
        return """<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>SIMPLR-AUV Mission Control</title>
    <script src="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.js"></script>
    <link rel="stylesheet" href="https://cdnjs.cloudflare.com/ajax/libs/leaflet/1.9.4/leaflet.min.css" />
    <style>
        * { margin: 0; padding: 0; box-sizing: border-box; }
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            background: linear-gradient(135deg, #1e3c72 0%, #2a5298 100%);
            color: #ffffff;
            height: 100vh;
            overflow: hidden;
        }
        .header {
            background: rgba(0, 0, 0, 0.3);
            padding: 15px 30px;
            border-bottom: 2px solid #00ff88;
            display: flex;
            justify-content: space-between;
            align-items: center;
        }
        .header h1 {
            font-size: 24px;
            font-weight: 300;
            color: #00ff88;
        }
        .connection-status {
            display: flex;
            align-items: center;
            gap: 10px;
        }
        .status-indicator {
            width: 12px;
            height: 12px;
            border-radius: 50%;
            background: #ff4444;
            transition: background 0.3s ease;
        }
        .status-indicator.connected {
            background: #00ff88;
            animation: pulse 2s infinite;
        }
        @keyframes pulse {
            0% { opacity: 1; }
            50% { opacity: 0.5; }
            100% { opacity: 1; }
        }
        .main-container {
            display: flex;
            height: calc(100vh - 80px);
        }
        .map-pane {
            flex: 1;
            position: relative;
            background: #1a1a1a;
        }
        #map {
            width: 100%;
            height: 100%;
            background: #0a1a2a;
        }
        .status-pane {
            width: 400px;
            background: rgba(0, 0, 0, 0.4);
            border-left: 2px solid #00ff88;
            overflow-y: auto;
            padding: 20px;
        }
        .status-section {
            background: rgba(255, 255, 255, 0.1);
            border-radius: 8px;
            padding: 15px;
            margin-bottom: 20px;
            border-left: 4px solid #00ff88;
        }
        .status-section h3 {
            color: #00ff88;
            margin-bottom: 10px;
            font-size: 16px;
            font-weight: 500;
        }
        .status-row {
            display: flex;
            justify-content: space-between;
            margin-bottom: 8px;
            font-size: 14px;
        }
        .status-value {
            color: #ffffff;
            font-weight: 500;
        }
    </style>
</head>
<body>
    <div class="header">
        <h1>🌊 SIMPLR-AUV Mission Control System</h1>
        <div class="connection-status">
            <div class="status-indicator" id="connectionIndicator"></div>
            <span id="connectionStatus">Connecting...</span>
        </div>
    </div>

    <div class="main-container">
        <div class="map-pane">
            <div id="map"></div>
        </div>

        <div class="status-pane">
            <div class="status-section">
                <h3>🎯 Mission Status</h3>
                <div class="status-row">
                    <span>Current Mode:</span>
                    <span class="status-value" id="currentMode">CONNECTING...</span>
                </div>
                <div class="status-row">
                    <span>Depth:</span>
                    <span class="status-value" id="depth">0.00m</span>
                </div>
                <div class="status-row">
                    <span>Safety Level:</span>
                    <span class="status-value" id="safetyLevel">NOMINAL</span>
                </div>
            </div>
        </div>
    </div>

    <script>
        const map = L.map('map').setView([34.0001, -117.0001], 16);
        
        L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}', {
            attribution: 'Tiles &copy; Esri'
        }).addTo(map);

        let socket = null;
        let vehicleMarker = null;

        function connectWebSocket() {
            socket = new WebSocket('ws://localhost:8765');
            
            socket.onopen = function(event) {
                console.log('WebSocket connected');
                document.getElementById('connectionIndicator').classList.add('connected');
                document.getElementById('connectionStatus').textContent = 'Connected';
            };
            
            socket.onmessage = function(event) {
                try {
                    const data = JSON.parse(event.data);
                    updateDisplay(data);
                } catch (error) {
                    console.error('Error parsing data:', error);
                }
            };
            
            socket.onclose = function(event) {
                console.log('WebSocket disconnected');
                document.getElementById('connectionIndicator').classList.remove('connected');
                document.getElementById('connectionStatus').textContent = 'Disconnected';
                setTimeout(connectWebSocket, 3000);
            };
        }

        function updateDisplay(data) {
            if (data.mission) {
                document.getElementById('currentMode').textContent = data.mission.mode || 'UNKNOWN';
            }
            if (data.position) {
                document.getElementById('depth').textContent = `${data.position.depth.toFixed(2)}m`;
                
                const lat = data.position.latitude;
                const lon = data.position.longitude;
                
                if (vehicleMarker) {
                    vehicleMarker.setLatLng([lat, lon]);
                } else {
                    vehicleMarker = L.marker([lat, lon]).addTo(map);
                    vehicleMarker.bindPopup('<b>SIMPLR-AUV</b><br>Real-time Position');
                }
            }
            if (data.health) {
                document.getElementById('safetyLevel').textContent = data.health.safety_level || 'NOMINAL';
            }
        }

        connectWebSocket();
    </script>
</body>
</html>"""
        
    def setup_gui_files(self):
        """Setup GUI files in gui/ directory"""
        gui_dir = Path("gui")
        gui_dir.mkdir(exist_ok=True)
        
        html_file = gui_dir / "index.html"
        with open(html_file, 'w', encoding='utf-8') as f:
            f.write(self.create_gui_html())
        
        print(f"[GUI] GUI files ready in {gui_dir.absolute()}")
        return gui_dir
    
    def start_http_server(self, gui_dir, port=8080):
        """Start HTTP server for GUI files"""
        class CustomHandler(SimpleHTTPRequestHandler):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=str(gui_dir), **kwargs)
            
            def log_message(self, format, *args):
                pass
        
        self.http_server = HTTPServer(('localhost', port), CustomHandler)
        
        def run_server():
            print(f"[GUI] HTTP server started on http://localhost:{port}")
            self.http_server.serve_forever()
        
        self.gui_thread = threading.Thread(target=run_server, daemon=True)
        self.gui_thread.start()
        
        return f"http://localhost:{port}"
    
    async def start_websocket_server(self):
        """Start WebSocket server for real-time data"""
        if self.websocket_server:
            try:
                server = await self.websocket_server.start_server()
                if server:
                    print(f"[GUI] WebSocket server successfully started")
                    return True
                else:
                    print(f"[GUI] Failed to start WebSocket server")
                    return False
            except Exception as e:
                print(f"[GUI] WebSocket server error: {e}")
                return False
        return False
    
    def extract_mission_data(self, nav_state, executive_commands, safety_status, 
                           ballast_status, mission_status, time_s):
        """Extract and format mission data for GUI"""
        
        if self.mission_start_time is None:
            self.mission_start_time = time_s
        
        # Extract position data
        position = nav_state.get('position', {}) if nav_state else {}
        lat = position.get('latitude', 34.0001)
        lon = position.get('longitude', -117.0001)
        depth = max(0.0, position.get('depth', 0.0))
        
        # Extract mission data
        mission_mode = 'INITIALIZE'
        if mission_status:
            mission_mode = mission_status.get('current_mode', 'INITIALIZE')
        elif executive_commands:
            mission_mode = executive_commands.get('mode', 'INITIALIZE')
        
        # Extract safety data
        safety_level = 'NOMINAL'
        if safety_status:
            safety_level_raw = safety_status.get('safety_level', 'NOMINAL')
            if hasattr(safety_level_raw, 'value'):
                safety_level = safety_level_raw.value
            else:
                safety_level = str(safety_level_raw)
        
        data = {
            'timestamp': time_s,
            'mission_time': time_s - self.mission_start_time,
            'position': {
                'latitude': float(lat),
                'longitude': float(lon),
                'depth': float(depth)
            },
            'mission': {
                'mode': str(mission_mode).replace('_', ' ').upper()
            },
            'health': {
                'safety_level': str(safety_level).upper()
            }
        }
        
        return data
    
    async def update_gui(self, nav_state, executive_commands, safety_status, 
                        ballast_status, mission_status, time_s):
        """Update GUI with current mission data"""
        if not self.websocket_server:
            return
            
        try:
            data = self.extract_mission_data(
                nav_state, executive_commands, safety_status, 
                ballast_status, mission_status, time_s
            )
            
            await self.websocket_server.broadcast_data(data)
            
        except Exception as e:
            print(f"[GUI] Error updating GUI: {e}")
    
    def launch_gui(self, auto_open=True):
        """Launch the complete GUI system"""
        try:
            gui_dir = self.setup_gui_files()
            gui_url = self.start_http_server(gui_dir)
            
            print(f"[GUI] Mission Control GUI available at: {gui_url}")
            if self.websocket_server:
                print(f"[GUI] WebSocket endpoint: ws://localhost:{self.websocket_server.port}")
            
            if auto_open:
                def open_browser():
                    time.sleep(3)
                    print(f"[GUI] Opening browser to {gui_url}")
                    webbrowser.open(gui_url)
                
                browser_thread = threading.Thread(target=open_browser, daemon=True)
                browser_thread.start()
                
            return gui_url
            
        except Exception as e:
            print(f"[GUI] Error launching GUI: {e}")
            return None

class SIMPLRAUVSystem:
    """Main SIMPLR-AUV system with proper surface start"""
    
    def __init__(self, use_simulation: bool = True, enable_gui: bool = True):
        """Initialize SIMPLR-AUV system"""
        self.use_simulation = use_simulation
        self.enable_gui = enable_gui and WEBSOCKETS_AVAILABLE
        self.is_running = False
        self.mission_active = False
        self._shutdown_event = asyncio.Event()
        
        set_simulation_mode(use_simulation)
        self._setup_logging()
        
        self.gui_manager = None
        if self.enable_gui:
            self.gui_manager = AUVGUIManager(self)
        
        try:
            self._initialize_subsystems()
            self.logger.info("[OK] All subsystems initialized successfully")
        except Exception as e:
            self.logger.error(f"[ERROR] Subsystem initialization failed: {e}")
            raise
        
        self.system_status = {
            'startup_time': time.time(),
            'last_update': 0.0,
            'update_count': 0,
            'errors': []
        }
        
        mode_text = "with GUI" if self.enable_gui else "without GUI"
        if not WEBSOCKETS_AVAILABLE and enable_gui:
            mode_text = "without GUI (websockets not installed)"
        
        self.logger.info(f"SIMPLR-AUV system ready (simulation={use_simulation}, {mode_text})")

    def _setup_logging(self):
        """Setup Windows 11 compatible logging"""
        try:
            log_dir = os.path.join(os.getcwd(), 'logs')
            os.makedirs(log_dir, exist_ok=True)
            
            logging.basicConfig(
                level=logging.INFO,
                format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
                handlers=[
                    logging.FileHandler(os.path.join(log_dir, 'simplr_auv.log'), encoding='utf-8'),
                    logging.StreamHandler(sys.stdout)
                ]
            )
            self.logger = logging.getLogger("SIMPLRAUVSystem")
            
        except Exception as e:
            print(f"Logging setup failed: {e}")
            logging.basicConfig(level=logging.INFO)
            self.logger = logging.getLogger("SIMPLRAUVSystem")

    def _initialize_subsystems(self):
        """Initialize all subsystems"""
        
        self.navigation_system = NavigationSystem(
            update_rate=10.0,
            gps_timeout=30.0,
            max_uncertainty=50.0
        )
        
        self.guidance_system = GuidanceSystem(
            default_speed=1.0,
            waypoint_tolerance=10.0,
            max_speed=2.5,
            min_speed=0.1
        )
        
        control_config = create_default_control_config()
        self.control_system = ControlSystem(control_config)
        
        self.ballast_tank = BallastTank(
            void_volume=0.070,
            max_buoyancy_change=4.37,
            fill_rate_lps=2.0,
            empty_rate_lps=2.5
        )
        
        self.actuator_module = ActuatorModule(
            xtail_max_angle=30.0,
            neutral_pwm=1500,
            pwm_range=400
        )
        
        self.executive_system = ExecutiveSystem()
        
        self.failsafe_system = FailsafeSystem(
            max_depth=20.0,
            min_battery_voltage=11.5,
            max_position_uncertainty=50.0,
            max_time_without_gps=1800.0,
            max_mission_duration=7200.0
        )

    async def start_system(self) -> bool:
        """Start system at surface"""
        try:
            if self.is_running:
                self.logger.warning("System already running")
                return True
            
            if not self.use_simulation:
                hardware_init_success = initialize_hardware()
                if not hardware_init_success:
                    self.logger.error("Hardware initialization failed")
                    return False
            
            if self.enable_gui and self.gui_manager:
                gui_url = self.gui_manager.launch_gui()
                if gui_url:
                    self.logger.info(f"[OK] Mission Control GUI launched: {gui_url}")
                
                ws_success = await self.gui_manager.start_websocket_server()
                if ws_success:
                    self.logger.info("[OK] WebSocket server ready for connections")
                else:
                    self.logger.warning("[WARNING] WebSocket server failed to start")
            
            await self.navigation_system.start()
            await self.control_system.start()
            
            self.is_running = True
            self.logger.info("[OK] SIMPLR-AUV system started at surface")
            return True
            
        except Exception as e:
            self.logger.error(f"[ERROR] System startup failed: {e}")
            return False

    async def stop_system(self):
        """Stop the SIMPLR-AUV system safely"""
        try:
            print("[STOP] Initiating system shutdown...")
            self.is_running = False
            self.mission_active = False
            self._shutdown_event.set()
            
            await self.navigation_system.stop()
            await self.control_system.stop()
            
            self.actuator_module.emergency_center_surfaces()
            self.ballast_tank.emergency_blow()
            
            if self.gui_manager:
                if self.gui_manager.websocket_server and self.gui_manager.websocket_server.server:
                    print("[STOP] Closing WebSocket server...")
                    self.gui_manager.websocket_server.server.close()
                    await self.gui_manager.websocket_server.server.wait_closed()
                
                if self.gui_manager.http_server:
                    print("[STOP] Stopping HTTP server...")
                    self.gui_manager.http_server.shutdown()
            
            self.logger.info("[OK] SIMPLR-AUV system stopped safely")
            
        except Exception as e:
            self.logger.error(f"[ERROR] System shutdown error: {e}")
            print(f"[ERROR] System shutdown error: {e}")

    def load_mission(self, mission_file: str = None) -> bool:
        """Load mission from file or use default"""
        try:
            if mission_file and os.path.exists(mission_file):
                mission_data = load_mission_from_file(mission_file)
                if not mission_data:
                    self.logger.warning(f"Failed to load {mission_file}, using default mission")
                    mission_data = create_default_mission()
            else:
                self.logger.info("Using default mission")
                mission_data = create_default_mission()
            
            success = self.executive_system.load_mission(mission_data)
            if success:
                self.logger.info("[OK] Mission loaded successfully")
            else:
                self.logger.error("[ERROR] Failed to load mission")
            
            return success
            
        except Exception as e:
            self.logger.error(f"[ERROR] Mission loading error: {e}")
            return False

    def start_mission(self) -> bool:
        """Start mission execution"""
        try:
            if not self.is_running:
                self.logger.error("Cannot start mission: system not running")
                return False
            
            success = self.executive_system.start_mission()
            if success:
                self.mission_active = True
                self.logger.info("[OK] Mission started at surface")
            else:
                self.logger.error("[ERROR] Failed to start mission")
            
            return success
            
        except Exception as e:
            self.logger.error(f"[ERROR] Mission start error: {e}")
            return False

    async def run_mission_loop(self, duration: Optional[float] = None, update_rate: float = 10.0):
        """Main mission execution loop"""
        if not self.is_running:
            self.logger.error("Cannot run mission: system not started")
            return
        
        loop_start_time = time.time()
        loop_interval = 1.0 / update_rate
        
        self.logger.info(f"[START] Mission loop at {update_rate}Hz")
        
        try:
            while self.is_running and not self._shutdown_event.is_set():
                if duration and (time.time() - loop_start_time) >= duration:
                    self.logger.info("Mission duration completed")
                    break
                
                iteration_start = time.time()
                await self._system_update_cycle()
                
                iteration_time = time.time() - iteration_start
                sleep_time = max(0, loop_interval - iteration_time)
                
                if sleep_time > 0:
                    try:
                        await asyncio.wait_for(
                            self._shutdown_event.wait(), 
                            timeout=sleep_time
                        )
                        break
                    except asyncio.TimeoutError:
                        pass
                elif iteration_time > loop_interval * 1.5:
                    self.logger.warning(f"[WARNING] Slow cycle: {iteration_time:.3f}s")
                
                self.system_status['update_count'] += 1
                
        except KeyboardInterrupt:
            self.logger.info("[STOP] Mission loop interrupted by user")
        except Exception as e:
            self.logger.error(f"[ERROR] Mission loop error: {e}")
            self.logger.error(traceback.format_exc())
        finally:
            await self.stop_system()

    async def _system_update_cycle(self):
        """Execute one complete system update cycle"""
        try:
            current_time = time.time()
            mission_time = current_time - self.system_status['startup_time']
            
            sensor_data = read_sensors()
            nav_state = self.navigation_system.update(mission_time)
            
            ballast_status = self.ballast_tank.get_ballast_status()
            
            vehicle_state = {
                'ballast': ballast_status,
                'actuators': self.actuator_module.get_actuator_status()
            }
            
            safety_status = self.failsafe_system.check_safety(
                nav_state, sensor_data, vehicle_state, mission_time
            )
            
            executive_commands = self.executive_system.execute(
                nav_state, sensor_data, safety_status, mission_time
            )
            
            # Process ballast commands through ballast tank
            ballast_command = executive_commands.get('ballast_command', 'EMPTY')
            ballast_status = self.ballast_tank.update(ballast_command, 0.1)
            
            guidance_commands = self.guidance_system.update(
                executive_commands, nav_state, mission_time
            )
            
            control_commands = self.control_system.update(
                guidance_commands, nav_state, 0.1
            )
            
            actuator_commands = self.actuator_module.update(control_commands, 0.1)
            send_actuator_commands(actuator_commands)
            
            if self.enable_gui and self.gui_manager:
                mission_status = self.executive_system.get_mission_status()
                await self.gui_manager.update_gui(
                    nav_state, executive_commands, safety_status, 
                    ballast_status, mission_status, mission_time
                )
            
            self.system_status['last_update'] = current_time
            
            if self.system_status['update_count'] % 100 == 0:
                self._log_system_status(nav_state, safety_status, executive_commands)
            
        except Exception as e:
            self.logger.error(f"[ERROR] System update error: {e}")
            self.system_status['errors'].append({
                'time': time.time(),
                'error': str(e)
            })

    def _log_system_status(self, nav_state: Dict[str, Any], safety_status: Dict[str, Any], 
                          executive_commands: Dict[str, Any]):
        """Log periodic system status"""
        try:
            mission_status = self.executive_system.get_mission_status()
            
            position = nav_state.get('position', {})
            lat = position.get('latitude', 34.0001)
            lon = position.get('longitude', -117.0001)
            depth = position.get('depth', 0.0)
            
            status_msg = (
                f"[STATUS] System Status - "
                f"Mode: {executive_commands.get('mode', 'unknown')}, "
                f"Position: {lat:.6f},{lon:.6f},{depth:.1f}m, "
                f"Safety: {safety_status.get('safety_level', 'unknown')}, "
                f"Progress: {mission_status.get('mission_progress', 0):.1f}%"
            )
            
            self.logger.info(status_msg)
            
        except Exception as e:
            self.logger.error(f"[ERROR] Status logging error: {e}")

    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status"""
        try:
            return {
                'system': {
                    'running': self.is_running,
                    'mission_active': self.mission_active,
                    'simulation_mode': self.use_simulation,
                    'gui_enabled': self.enable_gui,
                    'uptime': time.time() - self.system_status['startup_time'],
                    'update_count': self.system_status['update_count'],
                    'error_count': len(self.system_status['errors'])
                },
                'navigation': self.navigation_system.get_diagnostic_info(),
                'guidance': self.guidance_system.get_guidance_status(),
                'control': self.control_system.get_control_status(),
                'ballast': self.ballast_tank.get_ballast_status(),
                'actuators': self.actuator_module.get_actuator_status(),
                'mission': self.executive_system.get_mission_status(),
                'safety': self.failsafe_system.get_safety_statistics()
            }
        except Exception as e:
            self.logger.error(f"[ERROR] Status retrieval error: {e}")
            return {'error': str(e)}

    def emergency_surface(self):
        """Emergency surface procedure"""
        try:
            self.logger.critical("[EMERGENCY] Emergency surface initiated")
            self.failsafe_system.initiate_emergency_surface("Manual emergency surface")
            self.ballast_tank.emergency_blow()
            self.actuator_module.emergency_center_surfaces()
        except Exception as e:
            self.logger.error(f"[ERROR] Emergency surface error: {e}")

async def main_simulation(mission_file: str = None, duration: float = None, enable_gui: bool = True):
    """Main simulation entry point with proper surface start"""
    if not import_auv_modules():
        return False
    
    auv_system = None
    
    def signal_handler(signum, frame):
        """Handle Ctrl+C gracefully"""
        print(f"\n[STOP] Received signal {signum}, shutting down...")
        if auv_system:
            auv_system._shutdown_event.set()
    
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    try:
        print("=" * 60)
        print("SIMPLR-AUV Autonomous Underwater Vehicle System")
        print("Windows 11 Compatible Simulation Mode")
        print("CORRECTED: Surface start, proper GPS/depth logic")
        if enable_gui and WEBSOCKETS_AVAILABLE:
            print("Enhanced with Real-Time Web GUI")
        elif enable_gui and not WEBSOCKETS_AVAILABLE:
            print("GUI disabled - websockets package not installed")
        print("=" * 60)
        
        auv_system = SIMPLRAUVSystem(use_simulation=True, enable_gui=enable_gui)
        
        if not await auv_system.start_system():
            print("[ERROR] Failed to start AUV system")
            return False
        
        if not auv_system.load_mission(mission_file):
            print("[ERROR] Failed to load mission")
            return False
        
        if not auv_system.start_mission():
            print("[ERROR] Failed to start mission")
            return False
        
        gui_text = ""
        if auv_system.enable_gui:
            gui_text = " with Real-Time GUI"
            print("[INFO] Open your browser to view the Mission Control interface!")
            print("[INFO] The GUI should connect automatically within a few seconds")
            print("[INFO] Vehicle will start at surface with GPS available")
            print("[INFO] Check browser console (F12) for connection status")
            print(f"[INFO] WebSocket clients connected: {len(auv_system.gui_manager.websocket_server.clients)}")
        
        print(f"[START] Mission started at surface{gui_text} - Duration: {duration or 'unlimited'} seconds")
        print("Press Ctrl+C to stop...")
        print("")
        
        if auv_system.enable_gui:
            print("[INFO] Waiting 3 seconds for WebSocket server to be ready...")
            await asyncio.sleep(3)
            print(f"[INFO] WebSocket server ready. Clients: {len(auv_system.gui_manager.websocket_server.clients)}")
        
        await auv_system.run_mission_loop(duration=duration, update_rate=10.0)
        
        return True
        
    except KeyboardInterrupt:
        print("\n[STOP] Simulation interrupted by user")
        return True
    except Exception as e:
        print(f"[ERROR] Simulation error: {e}")
        traceback.print_exc()
        return False
    finally:
        if auv_system:
            print("[STOP] Final cleanup...")
            await auv_system.stop_system()
        print("[OK] Simulation ended")

if __name__ == "__main__":
    import argparse
    
    parser = argparse.ArgumentParser(description='SIMPLR-AUV System with Real-Time GUI')
    parser.add_argument('--mode', choices=['simulation', 'test', 'info'], 
                       default='simulation', help='Operation mode')
    parser.add_argument('--mission', type=str, help='Mission JSON file path')
    parser.add_argument('--duration', type=float, help='Simulation duration in seconds')
    parser.add_argument('--no-gui', action='store_true', help='Disable real-time GUI')
    parser.add_argument('--verbose', action='store_true', help='Enable verbose logging')
    
    args = parser.parse_args()
    
    if args.verbose:
        logging.getLogger().setLevel(logging.DEBUG)
    
    if args.mode == 'info':
        print("SIMPLR-AUV System Information")
        print("Real-time GUI available with websockets package")
        print("CORRECTED: Surface start, proper GPS/depth integration")
    elif args.mode == 'test':
        print("Running system tests...")
        if import_auv_modules():
            print("All modules imported successfully - System ready!")
        else:
            print("Module import failed")
    elif args.mode == 'simulation':
        try:
            success = asyncio.run(main_simulation(
                args.mission, 
                args.duration, 
                enable_gui=not args.no_gui
            ))
            sys.exit(0 if success else 1)
        except KeyboardInterrupt:
            print("\n[STOP] Interrupted")
            sys.exit(0)
    else:
        parser.print_help()
        sys.exit(1)