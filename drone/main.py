from pymavlink import mavutil
import time, serial

# Cellular Module Settings

CM_PORT = '/dev/ttyUSB2'
CM_BAUD = 115200

# Drone Settings
DRONE_PORT = '/dev/ttyUSB1'



# Establish connection
master = mavutil.mavlink_connection(DRONE_PORT) # meant for raspberry pi
# connect_to_drone = mavutil.mavlink_connection('COM4', baud=57600) # meant for windows

print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received")
print("Heartbeat from system (system %u component %u)" % 
      (master.target_system, master.target_component))

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

def main():
      ser = connect_to_cellular()
      print("RSSI & BER: ", send_at_command(ser, "AT+CSQ"))
      print("RSRP, RSRQ, SINR", send_at_command(ser, "AT+CESQ"))
      print(" Serving Cell Info (Includes RSRP, RSRQ, SINR, Cell ID, PCI)", send_at_command(ser, "AT+SIMCOMATI"))
      ser.close()

if __name__ == "__main__":
      main()


