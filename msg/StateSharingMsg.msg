uint64 timestamp             # time since system boot, us
uint64 timestamp_real_time   # UTC/realtime epoch timestamp, us (0 if unavailable)
uint8 sysid                  # vehicle ID (populated from MAVLink header sysid)

uint16 data_flags            # Bitmask of actual telemetry included in this message

# Global position coordinates
int32 global_position_lat    # latitude in degE7
int32 global_position_lon    # longitude in degE7
float32 global_position_alt  # altitude AMSL (m)

# Attitude quaternion (default core telemetry along with position)
float32[4] q                 # Attitude quaternion (w, x, y, z)

# Dynamic kinematics & auxiliary (trailing fields for zero-truncation)
float32[3] velocity          # NED velocity (m/s)
float32[3] acceleration      # NED linear acceleration (m/s^2)
float32[4] aux               # Auxiliary application-defined fields

# TOPICS incoming_state_sharing outgoing_state_sharing
