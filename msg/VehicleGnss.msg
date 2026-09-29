# Selected GNSS solution
#
# Published by the sensors module for every sample of the selected receiver, or of the blended solution.
# Heading is on vehicle_gnss_heading.

uint64 timestamp        # [us] Time since system start
uint64 timestamp_sample # [us] Measurement time, corrected by SENS_GNSSn_DELAY or PPS. Use this, not receiver.timestamp_sample: same value, but only top-level timestamps are time-synced over uXRCE-DDS

SensorGnss receiver # The selected receiver's sensor_gnss sample with the corrected timestamp_sample, or the blended solution with device_id 0

float32[3] antenna_offset # [m] [@frame FRD] Antenna position of the selected receiver, or the blended antenna position (SENS_GNSSn_OFFX/Y/Z)
