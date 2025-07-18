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

# Listen for messages
while True:
    msg = connect_to_drone.recv_match(blocking=True)
    if msg:
        print(msg)
    time.sleep(0.1)
