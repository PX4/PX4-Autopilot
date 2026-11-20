uint64 timestamp             # time since system boot, us
uint64 sync_epoch_us         # real-time synchronization epoch, us (0 = immediate)

# Command enum
uint8 COMMAND_NONE = 0
uint8 COMMAND_START = 1
uint8 COMMAND_STOP = 2
uint8 COMMAND_UPDATE_PARAMS = 3

# Data flags bitmask enum (which information agents must share)
uint16 DATA_FLAG_POSITION     = 1   # Bit 0: Global Position (lat, lon, alt)
uint16 DATA_FLAG_VELOCITY     = 2   # Bit 1: NED ground velocity (vx, vy, vz)
uint16 DATA_FLAG_ATTITUDE     = 4   # Bit 2: Quaternion attitude (q)
uint16 DATA_FLAG_ACCELERATION = 8   # Bit 3: NED ground acceleration (ax, ay, az)
uint16 DATA_FLAG_AUX1         = 16  # Bit 4: Auxiliary field 1 (aux[0])
uint16 DATA_FLAG_AUX2         = 32  # Bit 5: Auxiliary field 2 (aux[1])
uint16 DATA_FLAG_AUX3         = 64  # Bit 6: Auxiliary field 3 (aux[2])
uint16 DATA_FLAG_AUX4         = 128 # Bit 7: Auxiliary field 4 (aux[3])

uint8 command
uint8 target_system          # 0 = broadcast to all nodes, or specific sysid
uint16 data_flags            # Bitmask defining which fields agents must share
float32 sharing_period       # seconds
float32 delay_start          # seconds

# TOPICS incoming_state_sharing_control outgoing_state_sharing_control
