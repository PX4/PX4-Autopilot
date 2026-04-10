# Support for UBX-RXM-RAWX mesurment messages

uint8 ORB_QUEUE_LENGTH = 8

uint64 timestamp            # [us] publish time
uint64 timestamp_sample     # [us] measurement time (from rcvTow/week)

uint32 device_id
uint8  nsats

# Per-signal observation (max 32)
uint8[32]  gnss_id          # UBX gnssId
uint8[32]  sv_id            # satellite SVID/PRN
uint8[32]  sig_id           # UBX sigId (signal code)
uint8[32]  freq_id          # GLONASS freq channel, else 0
float64[32] pseudorange     # [m]
float64[32] carrier_phase   # [cycles]
float32[32] doppler         # [Hz]
float32[32] cn0             # [dB-Hz]
uint16[32] locktime_ms      # [ms]
float32[32] pr_stdev        # [m]
float32[32] cp_stdev        # [cycles]
float32[32] do_stdev        # [Hz]
uint8[32]   flags           # see bit constants

# Flags bitmask for flags[i] (mapped from UBX trkStat)
uint8 FLAG_PR_VALID     = 1
uint8 FLAG_CP_VALID     = 2
uint8 FLAG_DO_VALID     = 4
uint8 FLAG_PR_USED      = 8
uint8 FLAG_SUB_HALF_CYC = 16
uint8 FLAG_HALF_CYCLE   = 32
uint8 FLAG_CODE_LOCK    = 64
uint8 FLAG_CARR_LOCK    = 128

# TOPICS sensor_gps_raw
