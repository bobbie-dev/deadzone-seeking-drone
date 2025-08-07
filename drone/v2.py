#!/usr/bin/env python3
"""
Mission-Triggered Cellular Data Logger for UAV
Automatically starts/stops cellular data collection based on mission state
Optimized for 915MHz telemetry transmission
"""

from pymavlink import mavutil
import time, json, struct, serial, threading, re, os
from datetime import datetime
from collections import deque
from enum import Enum

# Hardware Configuration
CM_PORT = '/dev/ttyUSB2'  # Cellular module port
CM_BAUD = 115200
DRONE_PORT = '/dev/ttyUSB1'  # Drone telemetry port

# Mission States
class MissionState(Enum):
    IDLE = "idle"
    ARMED = "armed" 
    MISSION_ACTIVE = "mission_active"
    RTL = "rtl"
    LANDING = "landing"
    EMERGENCY = "emergency"

class CellularDataLogger:
    def __init__(self):
        # Core system variables
        self.master = None
        self.running = True
        self.mission_active = False
        self.mission_state = MissionState.IDLE
        self.current_mission_id = None
        
        # Data storage
        self.cellular_data_log = []
        self.current_location = {"lat": None, "lon": None, "alt": None}
        self.packet_counter = 0
        
        # Transmission management
        self.transmission_queue = deque(maxlen=100)
        self.last_transmission = 0
        self.min_transmission_interval = 30  # seconds
        self.last_cell_id = None
        self.last_location = None
        
        # Mission tracking
        self.mission_start_time = None
        self.mission_waypoint_count = 0
        self.current_waypoint = 0
        self.auto_mode_detected = False
        
        # Data file management
        self.current_data_file = None
        self.data_directory = "cellular_missions"
        
        # Create data directory
        if not os.path.exists(self.data_directory):
            os.makedirs(self.data_directory)

    def initialize_connection(self):
        """Initialize MAVLink connection to drone"""
        try:
            print(f"Connecting to drone on {DRONE_PORT}...")
            self.master = mavutil.mavlink_connection(DRONE_PORT)
            
            print("Waiting for heartbeat...")
            self.master.wait_heartbeat()
            
            print("Heartbeat received!")
            print(f"Connected to system {self.master.target_system} component {self.master.target_component}")
            
            # Request mission count to understand mission structure
            self.request_mission_list()
            
            return True
            
        except Exception as e:
            print(f"Failed to connect to drone: {e}")
            return False

    def connect_to_cellular(self):
        """Connect to cellular module"""
        try:
            ser = serial.Serial(CM_PORT, CM_BAUD, timeout=1)
            if not ser.is_open:
                ser.open()
            print("Connected to cellular module")
            return ser
        except Exception as e:
            print(f"Failed to connect to cellular module: {e}")
            return None

    def send_at_command(self, ser, command, response_timeout=2):
        """Send AT command to cellular module"""
        try:
            ser.write((command + '\r').encode())
            time.sleep(response_timeout)
            response = ser.read_all().decode(errors='ignore')
            return response.strip()
        except Exception as e:
            print(f"AT command error: {e}")
            return ""

    def parse_cellular_data(self, rssi_ber_response, cesq_response, cell_info_response):
        """Parse AT command responses and extract key numeric values"""
        parsed_data = {
            'rssi': -999, 'ber': -999, 'rsrp': -999, 'rsrq': -999, 
            'sinr': -999, 'cell_id': -999, 'pci': -999
        }
        
        try:
            # Parse RSSI and BER from AT+CSQ response
            rssi_match = re.search(r'\+CSQ:\s*(\d+),(\d+)', rssi_ber_response)
            if rssi_match:
                rssi_raw = int(rssi_match.group(1))
                parsed_data['rssi'] = -113 + (rssi_raw * 2) if rssi_raw != 99 else -999
                parsed_data['ber'] = int(rssi_match.group(2))
            
            # Parse CESQ response for RSRP, RSRQ
            cesq_match = re.search(r'\+CESQ:\s*(\d+),(\d+),(\d+),(\d+),(\d+),(\d+)', cesq_response)
            if cesq_match:
                rsrp_raw = int(cesq_match.group(5))
                rsrq_raw = int(cesq_match.group(6))
                parsed_data['rsrp'] = -140 + rsrp_raw if rsrp_raw != 255 else -999
                parsed_data['rsrq'] = -19.5 + (rsrq_raw * 0.5) if rsrq_raw != 255 else -999
            
            # Parse cell info for Cell ID and PCI
            cell_match = re.search(r'Cell ID[:\s]*(\d+)', cell_info_response, re.IGNORECASE)
            if cell_match:
                parsed_data['cell_id'] = int(cell_match.group(1))
                
            pci_match = re.search(r'PCI[:\s]*(\d+)', cell_info_response, re.IGNORECASE)
            if pci_match:
                parsed_data['pci'] = int(pci_match.group(1))
                
        except Exception as e:
            print(f"Error parsing cellular data: {e}")
        
        return parsed_data

    def get_current_location(self):
        """Get current GPS location from drone"""
        try:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
            if msg:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7  
                alt = msg.alt / 1000
                
                self.current_location['lat'] = lat
                self.current_location['lon'] = lon
                self.current_location['alt'] = alt
                
                return {"latitude": lat, "longitude": lon, "altitude": alt}
        except Exception as e:
            print(f"Error getting GPS location: {e}")
        
        # Return last known location if available
        if all(val is not None for val in self.current_location.values()):
            return {
                "latitude": self.current_location['lat'],
                "longitude": self.current_location['lon'],
                "altitude": self.current_location['alt']
            }
        else:
            return {"latitude": 0.0, "longitude": 0.0, "altitude": 0.0}

    def create_ultra_compact_packet(self, location, cellular_data):
        """Create minimal packet for 915MHz transmission"""
        try:
            lat_int = int(location['latitude'] * 1e6)
            lon_int = int(location['longitude'] * 1e6)
            alt_compressed = max(0, min(65535, int(location['altitude'])))
            
            rssi_compressed = max(0, min(255, cellular_data['rssi'] + 150))
            rsrp_compressed = max(0, min(255, cellular_data['rsrp'] + 180))
            rsrq_compressed = max(0, min(255, int((cellular_data['rsrq'] + 20) * 4)))
            cell_id_compressed = cellular_data['cell_id'] % 65536
            
            self.packet_counter = (self.packet_counter + 1) % 65536
            
            packet = struct.pack('!iiHBBBHH',
                lat_int, lon_int, alt_compressed,
                rssi_compressed, rsrp_compressed, rsrq_compressed,
                cell_id_compressed, self.packet_counter
            )
            
            return packet
            
        except Exception as e:
            print(f"Error creating compact packet: {e}")
            return None

    def should_transmit(self, cellular_data, location):
        """Intelligent transmission decision"""
        current_time = time.time()
        
        # Always respect minimum interval
        if current_time - self.last_transmission < self.min_transmission_interval:
            return False
        
        # During mission, be more aggressive with transmission
        if self.mission_active:
            # Poor signal areas (critical for mapping)
            if cellular_data['rssi'] < -100 or cellular_data['rsrp'] < -110:
                return True
                
            # Cell tower changes
            if self.last_cell_id and cellular_data['cell_id'] != self.last_cell_id:
                return True
                
            # Significant location changes (every ~300 meters during mission)
            if self.last_location:
                distance = self.calculate_distance(location, self.last_location)
                if distance > 0.003:  # ~300 meters
                    return True
                    
            # Regular mission updates (every 2 minutes)
            if current_time - self.last_transmission > 120:
                return True
        
        # When not in mission, transmit less frequently
        else:
            if current_time - self.last_transmission > 300:  # 5 minutes
                return True
                
        return False

    def calculate_distance(self, loc1, loc2):
        """Simple distance calculation"""
        lat_diff = loc1['latitude'] - loc2['latitude']
        lon_diff = loc1['longitude'] - loc2['longitude']
        return (lat_diff**2 + lon_diff**2)**0.5

    def transmit_cellular_data(self, packet_data):
        """Transmit data via MAVLink"""
        try:
            # Send via statustext for reliability
            status_msg = f"CD:{self.packet_counter}:{len(packet_data)}"
            self.master.mav.statustext_send(
                mavutil.mavlink.MAV_SEVERITY_INFO,
                status_msg.encode('utf-8')[:50]
            )
            
            self.last_transmission = time.time()
            print(f"Transmitted cellular data packet #{self.packet_counter}")
            return True
            
        except Exception as e:
            print(f"Transmission failed: {e}")
            return False

    def collect_cellular_data(self):
        """Collect cellular data and optionally transmit"""
        if not self.mission_active:
            return None
            
        try:
            ser = self.connect_to_cellular()
            if not ser:
                return None
                
            location = self.get_current_location()
            
            # Get cellular information
            rssi_ber = self.send_at_command(ser, "AT+CSQ")
            rsrp_rsrq_sinr = self.send_at_command(ser, "AT+CESQ")
            cell_info = self.send_at_command(ser, "AT+SIMCOMATI")
            
            parsed_data = self.parse_cellular_data(rssi_ber, rsrp_rsrq_sinr, cell_info)
            
            # Create data entry with mission context
            data_entry = {
                "timestamp": datetime.now().isoformat(),
                "mission_id": self.current_mission_id,
                "mission_time": time.time() - self.mission_start_time if self.mission_start_time else 0,
                "waypoint": self.current_waypoint,
                "mission_state": self.mission_state.value,
                "packet_num": self.packet_counter + 1,
                "location": location,
                "cellular": parsed_data,
                "raw_responses": {
                    "rssi_ber": rssi_ber,
                    "rsrp_rsrq_sinr": rsrp_rsrq_sinr,
                    "cell_info": cell_info
                }
            }
            
            # Store locally
            self.cellular_data_log.append(data_entry)
            
            # Decide whether to transmit
            if self.should_transmit(parsed_data, location):
                packet = self.create_ultra_compact_packet(location, parsed_data)
                if packet:
                    success = self.transmit_cellular_data(packet)
                    if success:
                        self.last_cell_id = parsed_data['cell_id']
                        self.last_location = location.copy()
            
            print(f"Collected data: Lat {location['latitude']:.6f}, Lon {location['longitude']:.6f}, "
                  f"RSSI={parsed_data['rssi']}dBm, Cell_ID={parsed_data['cell_id']}, "
                  f"Mission: {self.mission_state.value}")
            
            ser.close()
            return data_entry
            
        except Exception as e:
            print(f"Error collecting cellular data: {e}")
            return None

    def request_mission_list(self):
        """Request mission list from autopilot"""
        try:
            self.master.mav.mission_request_list_send(
                self.master.target_system,
                self.master.target_component
            )
            print("Requested mission list from autopilot")
        except Exception as e:
            print(f"Error requesting mission list: {e}")

    def start_mission_logging(self, mission_id=None):
        """Start mission-based data logging"""
        if self.mission_active:
            print("Mission logging already active")
            return
            
        self.mission_active = True
        self.mission_start_time = time.time()
        self.current_mission_id = mission_id or int(time.time())
        self.cellular_data_log = []  # Reset log for new mission
        self.packet_counter = 0
        
        # Create mission-specific data file
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"mission_{self.current_mission_id}_{timestamp}.json"
        self.current_data_file = os.path.join(self.data_directory, filename)
        
        print(f"🚀 MISSION LOGGING STARTED - Mission ID: {self.current_mission_id}")
        print(f"📁 Data file: {self.current_data_file}")
        
        # Send mission start notification
        start_msg = f"CELL_LOG_START:{self.current_mission_id}"
        self.master.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            start_msg.encode('utf-8')[:50]
        )

    def stop_mission_logging(self):
        """Stop mission logging and save data"""
        if not self.mission_active:
            return
            
        self.mission_active = False
        mission_duration = time.time() - self.mission_start_time if self.mission_start_time else 0
        
        # Save final data
        self.save_mission_data()
        
        print(f"🏁 MISSION LOGGING STOPPED - Duration: {mission_duration:.1f}s")
        print(f"📊 Collected {len(self.cellular_data_log)} data points")
        print(f"💾 Data saved to: {self.current_data_file}")
        
        # Send mission stop notification
        stop_msg = f"CELL_LOG_STOP:{len(self.cellular_data_log)}"
        self.master.mav.statustext_send(
            mavutil.mavlink.MAV_SEVERITY_INFO,
            stop_msg.encode('utf-8')[:50]
        )
        
        # Reset mission state
        self.mission_state = MissionState.IDLE
        self.current_waypoint = 0

    def save_mission_data(self):
        """Save collected data to file"""
        if not self.cellular_data_log or not self.current_data_file:
            return
            
        try:
            mission_summary = {
                "mission_info": {
                    "mission_id": self.current_mission_id,
                    "start_time": datetime.fromtimestamp(self.mission_start_time).isoformat() if self.mission_start_time else None,
                    "duration_seconds": time.time() - self.mission_start_time if self.mission_start_time else 0,
                    "total_data_points": len(self.cellular_data_log),
                    "waypoint_count": self.mission_waypoint_count,
                    "data_file": os.path.basename(self.current_data_file)
                },
                "cellular_data": self.cellular_data_log
            }
            
            with open(self.current_data_file, 'w') as f:
                json.dump(mission_summary, f, indent=2)
                
            print(f"Mission data saved successfully: {len(self.cellular_data_log)} entries")
            
        except Exception as e:
            print(f"Error saving mission data: {e}")

    def mavlink_message_handler(self):
        """Handle incoming MAVLink messages"""
        print("MAVLink message handler started...")
        
        while self.running:
            try:
                msg = self.master.recv_match(
                    type=['HEARTBEAT', 'MISSION_COUNT', 'MISSION_CURRENT', 'STATUSTEXT', 
                          'GLOBAL_POSITION_INT', 'SYS_STATUS'], 
                    blocking=True, 
                    timeout=1
                )
                
                if not msg:
                    continue
                    
                msg_type = msg.get_type()
                
                # Handle heartbeat and system state
                if msg_type == 'HEARTBEAT':
                    self.handle_heartbeat(msg)
                    
                elif msg_type == 'MISSION_COUNT':
                    self.mission_waypoint_count = msg.count
                    print(f"Mission has {self.mission_waypoint_count} waypoints")
                    
                elif msg_type == 'MISSION_CURRENT':
                    self.current_waypoint = msg.seq
                    
                elif msg_type == 'GLOBAL_POSITION_INT':
                    # Update current location
                    self.current_location['lat'] = msg.lat / 1e7
                    self.current_location['lon'] = msg.lon / 1e7
                    self.current_location['alt'] = msg.alt / 1000
                    
                elif msg_type == 'STATUSTEXT':
                    self.handle_statustext(msg)
                    
            except Exception as e:
                if self.running:
                    print(f"Error in MAVLink handler: {e}")
                time.sleep(0.1)

    def handle_heartbeat(self, msg):
        """Process heartbeat messages to detect mission state changes"""
        current_mode = msg.custom_mode
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        
        # Detect AUTO mode (mission execution)
        if current_mode in [3, 4, 6]:  # AUTO, GUIDED, RTL modes (adjust for your autopilot)
            if not self.auto_mode_detected and armed:
                self.auto_mode_detected = True
                if not self.mission_active:
                    print("🎯 AUTO mode detected - Starting mission logging")
                    self.start_mission_logging()
                    self.mission_state = MissionState.MISSION_ACTIVE
        else:
            if self.auto_mode_detected:
                self.auto_mode_detected = False
                if self.mission_active:
                    print("📍 AUTO mode ended - Stopping mission logging")
                    self.stop_mission_logging()

    def handle_statustext(self, msg):
        """Handle status text messages"""
        try:
            message_text = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else msg.text
            message_text = message_text.strip()
            
            # Handle manual commands
            if message_text == "START_CELL_LOG":
                print("Manual start command received")
                self.start_mission_logging()
                
            elif message_text == "STOP_CELL_LOG":
                print("Manual stop command received")
                self.stop_mission_logging()
                
            elif message_text == "DUMP_CELL_DATA":
                print("Data dump command received")
                if self.mission_active:
                    self.collect_cellular_data()
                self.save_mission_data()
                
        except Exception as e:
            print(f"Error handling status text: {e}")

    def periodic_data_collection(self):
        """Periodic cellular data collection during missions"""
        print("Periodic data collection thread started...")
        
        while self.running:
            try:
                if self.mission_active:
                    self.collect_cellular_data()
                    time.sleep(30)  # Collect every 30 seconds during mission
                else:
                    time.sleep(5)   # Check mission state every 5 seconds when idle
                    
            except Exception as e:
                if self.running:
                    print(f"Error in periodic data collection: {e}")
                time.sleep(1)

    def run(self):
        """Main execution loop"""
        print("="*60)
        print("🛰️  MISSION-TRIGGERED CELLULAR DATA LOGGER")
        print("="*60)
        
        # Initialize connections
        if not self.initialize_connection():
            print("❌ Failed to initialize drone connection")
            return False
            
        try:
            # Start background threads
            message_thread = threading.Thread(target=self.mavlink_message_handler, daemon=True)
            message_thread.start()
            
            collector_thread = threading.Thread(target=self.periodic_data_collection, daemon=True)
            collector_thread.start()
            
            print("✅ System initialized successfully")
            print("📡 Monitoring for mission start...")
            print("🎮 Manual commands: START_CELL_LOG, STOP_CELL_LOG, DUMP_CELL_DATA")
            print("⏹️  Press Ctrl+C to stop")
            print("-"*60)
            
            # Main loop
            while True:
                time.sleep(1)
                
                # Periodic status update
                if int(time.time()) % 60 == 0:  # Every minute
                    if self.mission_active:
                        duration = time.time() - self.mission_start_time
                        print(f"📊 Mission active: {duration:.0f}s, {len(self.cellular_data_log)} data points")
                    else:
                        print("⏳ Waiting for mission start...")
                
        except KeyboardInterrupt:
            print("\n🛑 Shutdown requested...")
            self.running = False
            
            # Save any active mission data
            if self.mission_active:
                print("💾 Saving final mission data...")
                self.stop_mission_logging()
            
            print("✅ Cellular data logger stopped")
            time.sleep(2)
            
        except Exception as e:
            print(f"❌ Critical error: {e}")
            self.running = False
            
        return True

# Main execution
if __name__ == "__main__":
    logger = CellularDataLogger()
    logger.run()