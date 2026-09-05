from pymavlink import mavutil

print("Starting MAVLink listener...")
# Try 14540 first, if it hangs, switch to 14550
connection_string = 'udpin:127.0.0.1:14550'
print(f"Connecting to {connection_string}...")

connection = mavutil.mavlink_connection(connection_string)

print("Waiting for heartbeat from PX4 SITL...")
connection.wait_heartbeat()

print(f"SUCCESS! Heartbeat received from system {connection.target_system} component {connection.target_component}")
# Add this below your heartbeat check to read live data
while True:
    msg = connection.recv_match(type='DEBUG_VECT', blocking=True)
    if msg:
        print(msg)