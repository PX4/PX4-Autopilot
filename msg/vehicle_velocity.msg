# body frame velocity and acceleration

uint64 timestamp                        # time since system start (microseconds)
uint64 timestamp_sample                 # the timestamp of the raw data (microseconds)

float32[3] velocity                     # velocity in body frame, (metres/sec)

float32[3] acceleration                 # velocity derivative in body frame, (metres/sec^2)
