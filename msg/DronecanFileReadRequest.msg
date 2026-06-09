# Request to read a file from a DroneCAN (UAVCAN v0) file server, proxied by the
# uavcannode file-read client over uORB. Published by a driver (the session
# master), consumed by uavcannode which issues the corresponding file.GetInfo /
# file.Read service call. Vendor-neutral: any cannode driver can use this to pull
# a file (firmware image, config, calibration) from the flight controller.

uint64 timestamp                # time since system start (microseconds)

uint32 session_id               # increments per transfer; echoed in the response so stale replies are dropped
uint32 offset                   # byte offset to read from (CMD_READ)
uint8  cmd                      # one of CMD_*
uint8  server_node_id           # file server node id; 0 = auto/discover
char[128] path                  # version-gated query path, e.g. "module/ublox-f9p@1.32"

uint8 CMD_GETINFO = 0           # query file size / existence
uint8 CMD_READ = 1              # read a chunk at offset
uint8 CMD_ABORT = 2             # cancel the in-flight transfer

uint8 ORB_QUEUE_LENGTH = 4
