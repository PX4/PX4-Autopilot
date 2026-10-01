# Selected GNSS solution
#
# Published by the sensors module for every sample of the selected receiver, usable or not, with the selection state and the check result for that sample.
# Heading is on vehicle_gnss_heading.

uint64 timestamp        # [us] Time since system start
uint64 timestamp_sample # [us] Measurement time, delay corrected. Use this rather than receiver.timestamp_sample, which holds the same value

SensorGnss receiver # The selected receiver's sensor_gnss sample with the corrected timestamp_sample

float32[3] antenna_offset # [m] [@frame FRD] Antenna position of the selected receiver

# Selection state
uint8 selected_instance       # [-] sensor_gnss instance of the selected receiver
uint8 selection_count         # [-] Increments when the selected receiver changes
uint8 selection_reason        # [@enum SELECTION] Why this receiver is selected
uint8 SELECTION_PREFERRED    = 0 # The preferred receiver
uint8 SELECTION_ONLY         = 1 # No other receiver has published
uint8 SELECTION_RANKED       = 2 # No receiver is preferred, and no other one ranks higher
uint8 SELECTION_TIMEOUT      = 3 # The previously selected or the preferred receiver stopped publishing
uint8 SELECTION_UNHEALTHY    = 4 # The previously selected or the preferred receiver failed its checks or delivered samples late, continuously or often
uint8 SELECTION_REQUIREMENTS = 5 # Meets the accuracy requirements, which the previously selected receiver didn't
uint8 SELECTION_RTK_FIXED    = 6 # Has an RTK fixed solution, which the previously selected receiver didn't, and both meet the accuracy requirements

# Check result for this sample
bool usable                 # Sample may be used: the receiver has passed its checks for long enough
uint16 failed_checks        # [@enum CHECK] Bitmask of the enabled checks that failed
uint16 CHECK_NSATS   = 1    # Too few satellites
uint16 CHECK_PDOP    = 2    # Position dilution of precision too high
uint16 CHECK_EPH     = 4    # Horizontal position accuracy too low
uint16 CHECK_EPV     = 8    # Vertical position accuracy too low
uint16 CHECK_SACC    = 16   # Speed accuracy too low
uint16 CHECK_HDRIFT  = 32   # Horizontal position drift at rest too high
uint16 CHECK_VDRIFT  = 64   # Vertical position drift at rest too high
uint16 CHECK_HSPEED  = 128  # Horizontal speed at rest too high
uint16 CHECK_VSPEED  = 256  # Vertical speed at rest too high
uint16 CHECK_SPOOFED = 512  # Receiver reports spoofing
uint16 CHECK_FIX     = 1024 # Fix type too low
uint16 CHECK_JAMMED  = 2048 # Receiver reports jamming
