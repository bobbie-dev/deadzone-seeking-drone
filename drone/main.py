from pymavlink import mavutil
import time, json, struct, serial, threading

# Cellular Module Settings
CM_PORT = '/dev/ttyUSB2'
CM_BAUD = 115200
# Drone Settings
DRONE_PORT = '/dev/ttyUSB1'

# Establish connection
master = mavutil.mavlink_connection(DRONE_PORT)  # meant for raspberry pi
# connect_to_drone = mavutil.mavlink_connection('COM4', baud=57600)  # meant for windows
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received")
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

# Global variables for data storage
cellular_data_log = []
running = True
current_location = {"lat": None, "lon": None, "alt": None}

def connect_to_cellular():
    ser = serial.Serial(CM_PORT, CM_BAUD, timeout=1)
    if not ser.is_open:
        ser.open()
    print("Connected to cellular module")
    return ser

def send_at_command(ser, command, response_timeout=2):
    ser.write(command + '\r'.encode())
    time.sleep(response_timeout)
    response = ser.read_all().decode(errors='ignore')
    return response.strip()

def store_drone_information(packet_num, rssi, rsrp, rsrq, sinr, cell_id, pci):
    msg = struct.pack('!7i', packet_num, rssi, rsrp, rsrq, sinr, cell_id, pci)
    master.mav.send(
        mavutil.mavlink.MAVLink_custommsg_message(
            Packet=f"{packet_num}".encode(),
            Data=msg,
        )
    )

def get_current_location():
    """Get current GPS location from drone"""
    try:
        # Request a GPS position message
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7  
            alt = msg.alt / 1000  # Convert from mm to m
            
            # Update global location
            current_location['lat'] = lat
            current_location['lon'] = lon
            current_location['alt'] = alt
            
            return {
                "latitude": lat,
                "longitude": lon, 
                "altitude": alt
            }
    except Exception as e:
        print(f"Error getting GPS location: {e}")
    
    # Return last known location if current request fails
    if all(val is not None for val in current_location.values()):
        return {
            "latitude": current_location['lat'],
            "longitude": current_location['lon'],
            "altitude": current_location['alt']
        }
    else:
        return {
            "latitude": "Unknown",
            "longitude": "Unknown", 
            "altitude": "Unknown"
        }

def collect_cellular_data():
def collect_cellular_data():
    """Collect cellular data and store it with GPS location"""
    try:
        ser = connect_to_cellular()
        
        # Get current location
        location = get_current_location()
        
        # Get cellular information
        rssi_ber = send_at_command(ser, "AT+CSQ")
        rsrp_rsrq_sinr = send_at_command(ser, "AT+CESQ")
        cell_info = send_at_command(ser, "AT+SIMCOMATI")
        
        # Create data entry with location
        data_entry = {
            "location": location,
            "rssi_ber": rssi_ber,
            "rsrp_rsrq_sinr": rsrp_rsrq_sinr,
            "cell_info": cell_info
        }
        
        # Store in local log
        cellular_data_log.append(data_entry)
        
        print(f"Data collected at location: Lat {location['latitude']}, Lon {location['longitude']}, Alt {location['altitude']}")
        print(f"RSSI & BER: {rssi_ber}")
        print(f"RSRP, RSRQ, SINR: {rsrp_rsrq_sinr}")
        print(f"Serving Cell Info: {cell_info}")
        
        ser.close()
        return data_entry
        
    except Exception as e:
        print(f"Error collecting cellular data: {e}")
        return None

def dump_data_to_file():
    """Dump all collected data to a JSON file"""
    try:
        # Use current location and entry count for filename
        loc = get_current_location()
        lat_str = f"{loc['latitude']:.6f}".replace('.', 'd') if isinstance(loc['latitude'], float) else "unknown"
        lon_str = f"{loc['longitude']:.6f}".replace('.', 'd') if isinstance(loc['longitude'], float) else "unknown" 
        filename = f"cellular_data_dump_lat{lat_str}_lon{lon_str}_{len(cellular_data_log)}entries.json"
        
        with open(filename, 'w') as f:
            json.dump(cellular_data_log, f, indent=2)
        
        print(f"Data dumped to {filename}")
        print(f"Total entries: {len(cellular_data_log)}")
        
        # Optionally clear the log after dumping
        # cellular_data_log.clear()
        
        return filename
        
    except Exception as e:
        print(f"Error dumping data: {e}")
        return None

def mavlink_listener():
    """Listen for MAVLink messages, specifically DUMP_DATA commands and GPS updates"""
    print("MAVLink listener started...")
    
    while running:
        try:
            # Listen for multiple message types
            msg = master.recv_match(type=['STATUSTEXT', 'GLOBAL_POSITION_INT'], blocking=True, timeout=1)
            
            if msg:
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    # Update current location when GPS data is received
                    current_location['lat'] = msg.lat / 1e7
                    current_location['lon'] = msg.lon / 1e7
                    current_location['alt'] = msg.alt / 1000
                    
                elif msg.get_type() == 'STATUSTEXT':
                    message_text = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else msg.text
                    print(f"Received statustext: {message_text}")
                    
                    # Check if it's the DUMP_DATA command
                    if message_text.strip() == "DUMP_DATA":
                        print("DUMP_DATA command received! Initiating data dump...")
                        
                        # Collect current data
                        collect_cellular_data()
                        
                        # Dump all data to file
                        filename = dump_data_to_file()
                        
                        # Send confirmation back to drone
                        if filename:
                            confirmation = f"Data dumped to {filename}"
                        else:
                            confirmation = "Data dump failed"
                        
                        master.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO,
                            confirmation.encode('utf-8')[:50]  # MAVLink statustext has 50 char limit
                        )
                    
        except Exception as e:
            if running:  # Only print error if we're still supposed to be running
                print(f"Error in MAVLink listener: {e}")
            time.sleep(0.1)

def periodic_data_collection():
    """Periodically collect cellular data"""
    print("Periodic data collection started...")
    
    while running:
        try:
            collect_cellular_data()
            time.sleep(30)  # Collect data every 30 seconds
        except Exception as e:
            if running:
                print(f"Error in periodic data collection: {e}")
            time.sleep(1)

def main():
    global running
    
    try:
        # Start the MAVLink listener in a separate thread
        listener_thread = threading.Thread(target=mavlink_listener, daemon=True)
        listener_thread.start()
        
        # Start periodic data collection in a separate thread
        collector_thread = threading.Thread(target=periodic_data_collection, daemon=True)
        collector_thread.start()
        
        print("Ground base system started.")
        print("- MAVLink listener is active")
        print("- Periodic data collection is active")
        print("- Send 'DUMP_DATA' command from drone UI to trigger data dump")
        print("- Press Ctrl+C to stop")
        
        # Keep main thread alive
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down ground base system...")
        running = False
        
        # Give threads time to finish
        time.sleep(2)
        
        print("Ground base system stopped.")

if __name__ == "__main__":
    main()