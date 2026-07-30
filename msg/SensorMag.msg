uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 x                 # [Gauss] magnetic field in the FRD board frame X-axis
float32 y                 # [Gauss] magnetic field in the FRD board frame Y-axis
float32 z                 # [Gauss] magnetic field in the FRD board frame Z-axis

float32 temperature       # [°C] Temperature.

uint32 error_count

uint8 ORB_QUEUE_LENGTH = 4
