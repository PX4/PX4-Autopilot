import socket
import struct
from pymavlink import mavutil
import time

# UDP Configuration (Simulink -> Python)
UDP_IP = "192.168.1.154"  # Adjust as needed
UDP_PORT = 5005           # Match Simulink's send port

# MAVLink Connection (Python -> PX4 SITL)
master = mavutil.mavlink_connection("udp:127.0.0.1:14550")  # Adjust for real hardware if needed

# Wait for MAVLink heartbeat
print("Waiting for heartbeat from PX4...")
master.wait_heartbeat()
print("Connected to PX4!")

# Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for UDP packets on {UDP_IP}:{UDP_PORT}...")

while True:
    data, addr = sock.recvfrom(1024)  # Receive data (max 1024 bytes)
    
    if len(data) == 8:
        # Unpack 8 bytes as two float32 values (Ay and Az)
        Az_cmd, Ay_cmd = struct.unpack('<ff', data)  # Little-endian format
        
        print(f"Received {len(data)} bytes from {addr}: Ay={Ay_cmd:.3f}, Az={Az_cmd:.3f}")
        # Send MAVLink message to ArduPlane
        master.mav.commanded_accel_send(Az_cmd, Ay_cmd)
        print(f"Sent COMMANDED_ACCEL to ArduPlane: Az_cmd={Az_cmd}, Ay_cmd={Ay_cmd}")
    else:
        print(f"Unexpected packet size: {len(data)} bytes - Data: {data.hex()}")

