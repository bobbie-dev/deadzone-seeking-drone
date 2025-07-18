from pymavlink import mavutil
import time

# Establish connection
connect_to_drone = mavutil.mavlink_connection('COM4', baud=57600)

print("Waiting for heartbeat...")

# Wait for the first heartbeat 
connect_to_drone.wait_heartbeat()
print("Heartbeat received")
print("Heartbeat from system (system %u component %u)" % 
      (connect_to_drone.target_system, connect_to_drone.target_component))

# Listen for GPS messages only
while True:
    msg = connect_to_drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7   # Convert to decimal degrees
        lon = msg.lon / 1e7
        alt = msg.alt / 1000  # Convert from millimeters to meters
        print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt} m")
    time.sleep(0.1)
