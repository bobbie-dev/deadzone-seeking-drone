from pymavlink import mavutil
import time, json, struct, serial, threading, re

# Cellular Module Settings
CM_PORT = '/dev/ttyUSB2'
CM_BAUD = 115200
# Drone Settings
DRONE_PORT = '/dev/ttyUSB1'

# Establish connection
master = mavutil.mavlink_connection(DRONE_PORT)  # meant for raspberry pi
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received")
print("Heartbeat from system (system %u component %u)" %
      (master.target_system, master.target_component))

# Global variables for data storage
cellular_data_log = []
running = True
current_location = {"lat": None, "lon": None, "alt": None}
packet_counter = 0

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

def parse_cellular_data(rssi_ber_response, cesq_response, cell_info_response):
    """Parse AT command responses and extract key numeric values only"""
    parsed_data = {
        'rssi': -999, 'ber': -999, 'rsrp': -999, 'rsrq': -999, 
        'sinr': -999, 'cell_id': -999, 'pci': -999
    }
    
    try:
        # Parse RSSI and BER from AT+CSQ response
        # Expected format: "+CSQ: 15,99"
        rssi_match = re.search(r'\+CSQ:\s*(\d+),(\d+)', rssi_ber_response)
        if rssi_match:
            rssi_raw = int(rssi_match.group(1))
            parsed_data['rssi'] = -113 + (rssi_raw * 2) if rssi_raw != 99 else -999
            parsed_data['ber'] = int(rssi_match.group(2))
        
        # Parse CESQ response for RSRP, RSRQ, SINR
        # Expected format: "+CESQ: 99,99,255,255,20,80"
        cesq_match = re.search(r'\+CESQ:\s*(\d+),(\d+),(\d+),(\d+),(\d+),(\d+)', cesq_response)
        if cesq_match:
            rsrp_raw = int(cesq_match.group(5))
            rsrq_raw = int(cesq_match.group(6))
            parsed_data['rsrp'] = -140 + rsrp_raw if rsrp_raw != 255 else -999
            parsed_data['rsrq'] = -19.5 + (rsrq_raw * 0.5) if rsrq_raw != 255 else -999
        
        # Parse cell info for Cell ID and PCI (this varies by modem)
        # You'll need to adjust this regex based on your modem's response format
        cell_match = re.search(r'Cell ID[:\s]*(\d+)', cell_info_response, re.IGNORECASE)
        if cell_match:
            parsed_data['cell_id'] = int(cell_match.group(1))
            
        pci_match = re.search(r'PCI[:\s]*(\d+)', cell_info_response, re.IGNORECASE)
        if pci_match:
            parsed_data['pci'] = int(pci_match.group(1))
            
    except Exception as e:
        print(f"Error parsing cellular data: {e}")
    
    return parsed_data

def get_current_location():
    """Get current GPS location from drone"""
    try:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7  
            alt = msg.alt / 1000
            
            current_location['lat'] = lat
            current_location['lon'] = lon
            current_location['alt'] = alt
            
            return {"latitude": lat, "longitude": lon, "altitude": alt}
    except Exception as e:
        print(f"Error getting GPS location: {e}")
    
    if all(val is not None for val in current_location.values()):
        return {
            "latitude": current_location['lat'],
            "longitude": current_location['lon'],
            "altitude": current_location['alt']
        }
    else:
        return {"latitude": 0.0, "longitude": 0.0, "altitude": 0.0}

def send_compact_data_to_drone(location, parsed_cellular):
    """Send compact binary data to drone using MAVLink custom message"""
    global packet_counter
    packet_counter += 1
    
    try:
        # Pack data into compact binary format (32 bytes total - more complete)
        # Format: packet_num(4) + lat(4) + lon(4) + alt(4) + rssi(2) + rsrp(2) + rsrq(2) + sinr(2) + cell_id(4) + pci(2) + ber(2) + timestamp(4)
        lat_int = int(location['latitude'] * 1e7)  # Convert back to MAVLink format
        lon_int = int(location['longitude'] * 1e7)
        alt_int = int(location['altitude'] * 1000)
        timestamp = int(time.time())  # Unix timestamp
        
        packed_data = struct.pack('!iiihhhhhhhI', 
            packet_counter,
            lat_int,
            lon_int, 
            alt_int,
            parsed_cellular['rssi'],
            parsed_cellular['rsrp'],
            parsed_cellular['rsrq'],
            parsed_cellular['sinr'],
            parsed_cellular['cell_id'],
            parsed_cellular['pci'],
            parsed_cellular['ber'],
            timestamp
        )
        
        # Send via MAVLink custom message
        master.mav.send(
            mavutil.mavlink.MAVLink_custommsg_message(
                Packet=str(packet_counter).encode()[:10],  # Max 10 chars
                Data=packed_data[:200]  # Ensure within MAVLink limits
            )
        )
        
        print(f"Sent compact data packet #{packet_counter} ({len(packed_data)} bytes)")
        
    except Exception as e:
        print(f"Error sending compact data: {e}")

def collect_cellular_data():
    """Collect cellular data and store it with GPS location"""
    try:
        ser = connect_to_cellular()
        location = get_current_location()
        
        # Get cellular information
        rssi_ber = send_at_command(ser, "AT+CSQ")
        rsrp_rsrq_sinr = send_at_command(ser, "AT+CESQ")
        cell_info = send_at_command(ser, "AT+SIMCOMATI")
        
        # Parse into compact numeric format
        parsed_data = parse_cellular_data(rssi_ber, rsrp_rsrq_sinr, cell_info)
        
        # Create compact data entry
        data_entry = {
            "packet_num": packet_counter + 1,
            "location": location,
            "cellular": parsed_data,
            "raw_responses": {  # Keep raw for local logging only
                "rssi_ber": rssi_ber,
                "rsrp_rsrq_sinr": rsrp_rsrq_sinr,
                "cell_info": cell_info
            }
        }
        
        # Store locally
        cellular_data_log.append(data_entry)
        
        # Send compact version to drone
        send_compact_data_to_drone(location, parsed_data)
        
        print(f"Data collected at Lat {location['latitude']:.6f}, Lon {location['longitude']:.6f}")
        print(f"Parsed: RSSI={parsed_data['rssi']}dBm, RSRP={parsed_data['rsrp']}dBm, Cell_ID={parsed_data['cell_id']}")
        
        ser.close()
        return data_entry
        
    except Exception as e:
        print(f"Error collecting cellular data: {e}")
        return None

def dump_data_to_file():
    """Dump all collected data to a JSON file"""
    try:
        loc = get_current_location()
        lat_str = f"{loc['latitude']:.6f}".replace('.', 'd').replace('-', 'n')
        lon_str = f"{loc['longitude']:.6f}".replace('.', 'd').replace('-', 'w')
        filename = f"cellular_data_{lat_str}_{lon_str}_{len(cellular_data_log)}.json"
        
        with open(filename, 'w') as f:
            json.dump(cellular_data_log, f, indent=2)
        
        print(f"Data dumped to {filename} ({len(cellular_data_log)} entries)")
        return filename
        
    except Exception as e:
        print(f"Error dumping data: {e}")
        return None

def mavlink_listener():
    """Listen for MAVLink messages"""
    print("MAVLink listener started...")
    
    while running:
        try:
            msg = master.recv_match(type=['STATUSTEXT', 'GLOBAL_POSITION_INT'], blocking=True, timeout=1)
            
            if msg:
                if msg.get_type() == 'GLOBAL_POSITION_INT':
                    current_location['lat'] = msg.lat / 1e7
                    current_location['lon'] = msg.lon / 1e7
                    current_location['alt'] = msg.alt / 1000
                    
                elif msg.get_type() == 'STATUSTEXT':
                    message_text = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else msg.text
                    
                    if message_text.strip() == "DUMP_DATA":
                        print("DUMP_DATA command received!")
                        collect_cellular_data()
                        filename = dump_data_to_file()
                        
                        # Send short confirmation
                        confirmation = f"OK:{len(cellular_data_log)}" if filename else "ERR"
                        master.mav.statustext_send(
                            mavutil.mavlink.MAV_SEVERITY_INFO,
                            confirmation.encode('utf-8')[:50]
                        )
                    
        except Exception as e:
            if running:
                print(f"Error in MAVLink listener: {e}")
            time.sleep(0.1)

def periodic_data_collection():
    """Periodically collect and transmit cellular data"""
    print("Periodic data collection started...")
    
    while running:
        try:
            collect_cellular_data()
            time.sleep(60)  # Increased interval to reduce radio traffic
        except Exception as e:
            if running:
                print(f"Error in periodic data collection: {e}")
            time.sleep(1)

def main():
    global running
    
    try:
        listener_thread = threading.Thread(target=mavlink_listener, daemon=True)
        listener_thread.start()
        
        collector_thread = threading.Thread(target=periodic_data_collection, daemon=True)
        collector_thread.start()
        
        print("Optimized ground base system started for 915MHz")
        print("- Compact binary data transmission")
        print("- Reduced message frequency")
        print("- Local full logging, minimal radio traffic")
        print("- Press Ctrl+C to stop")
        
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\nShutting down...")
        running = False
        time.sleep(2)

if __name__ == "__main__":
    main()