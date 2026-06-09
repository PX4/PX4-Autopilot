# Response to a DronecanFileReadRequest. Published by the uavcannode file-read
# client from the file.GetInfo / file.Read service-call response, consumed by the
# requesting driver. session_id + offset let the driver match a reply to the
# outstanding request and ignore stale or duplicate responses.

uint64 timestamp                # time since system start (microseconds)

uint64 file_size                # total file size in bytes (from CMD_GETINFO)
uint32 session_id               # echoes the request
uint32 offset                   # echoes the request offset (CMD_READ)
uint16 len                      # number of valid bytes in data
int16  error                    # 0 = ok; >0 = DroneCAN file error or ERROR_NO_UPDATE
uint8  cmd                      # echoes the request cmd
uint8  eof                      # 1 if this is the last chunk (len < 256)
uint8[256] data                 # chunk payload (file.Read v0 cap is 256 bytes)

int16 ERROR_OK = 0
int16 ERROR_NO_UPDATE = 1000    # resolver: requested version is already newest (not a failure)

uint8 ORB_QUEUE_LENGTH = 4
