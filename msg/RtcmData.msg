# RTCM3 data exchanged with GNSS receivers.
#
# Published under two topic names that share this definition (see TOPICS below):
#
# rtcm_corrections - external fixed-base corrections fed into the vehicle (MAVLink
# GPS_RTCM_DATA, UAVCAN RTCMStream, GPS drivers in dump mode). Multiple
# sources are allowed, one uORB instance each; consumers select an instance
# via their stale-link logic.
#
# rtcm_moving_baseline - moving-base GPS output (RTCM 4072 or equivalent) intended for a rover.
# Single publisher per vehicle (on-board moving base, or a CANnode
# forwarding MovingBaselineData); consumers only read instance 0.

uint64 timestamp # [us] Time since system start

uint32 device_id # [-] Unique device ID of the publisher that produced this RTCM

uint16 len # [-] Length of data
uint8 flags # [-] LSB: 1=fragmented
uint8[300] data # Correction payload (fixed-base RTCM3 and/or SPARTN frames, or moving-baseline RTCM3)

uint8 ORB_QUEUE_LENGTH = 16

# Sized for the fixed-base corrections case (up to four independent sources). The moving-baseline
# topic only uses instance 0 (single publisher per vehicle).
uint8 MAX_INSTANCES = 4

# TOPICS rtcm_corrections rtcm_moving_baseline
