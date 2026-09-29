# Selected GNSS solution
#
# Published by the sensors module for every sample of the selected receiver, usable or not, with the check result for that sample.
# Heading is on vehicle_gnss_heading.

uint64 timestamp        # [us] Time since system start
uint64 timestamp_sample # [us] Measurement time, corrected by SENS_GNSSn_DELAY or PPS. Use this, not receiver.timestamp_sample: same value, but only top-level timestamps are time-synced over uXRCE-DDS

SensorGnss receiver # The selected receiver's sensor_gnss sample with the corrected timestamp_sample

float32[3] antenna_offset # [m] [@frame FRD] Antenna position of the selected receiver (SENS_GNSSn_OFFX/Y/Z)

# Check result for this sample
bool usable                 # Passes the checks enabled in GNSS_CHECK, and has for long enough (GNSS_REQ_TIME)
uint16 failed_checks        # [@enum CHECK] Failed checks among those enabled in GNSS_CHECK, in GNSS_CHECK bit order
uint16 CHECK_NSATS   = 1    # Satellites below GNSS_REQ_NSATS
uint16 CHECK_PDOP    = 2    # PDOP above GNSS_REQ_PDOP
uint16 CHECK_EPH     = 4    # eph above GNSS_REQ_EPH, 50 m when relaxed
uint16 CHECK_EPV     = 8    # epv above GNSS_REQ_EPV, 50 m when relaxed
uint16 CHECK_SACC    = 16   # Speed accuracy above GNSS_REQ_SACC, 10 m/s when relaxed
uint16 CHECK_HDRIFT  = 32   # Horizontal drift at rest above GNSS_REQ_HDRIFT
uint16 CHECK_VDRIFT  = 64   # Vertical drift at rest above GNSS_REQ_VDRIFT
uint16 CHECK_HSPEED  = 128  # Horizontal speed at rest above GNSS_REQ_HDRIFT
uint16 CHECK_VSPEED  = 256  # Vertical speed at rest above GNSS_REQ_VDRIFT
uint16 CHECK_SPOOFED = 512  # Receiver reports spoofing
uint16 CHECK_FIX     = 1024 # Fix below GNSS_REQ_FIX, 3D when relaxed
uint16 CHECK_JAMMED  = 2048 # Receiver reports jamming
