uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 dt           # delta time between samples (microseconds)
uint8 samples        # number of valid samples

float32 accel_scale
float32 gyro_scale

uint8 FIFO_SIZE = 24

int16[24] accel_x         # gyro in the FRD board frame X-axis
int16[24] accel_y         # gyro in the FRD board frame Y-axis
int16[24] accel_z         # gyro in the FRD board frame Z-axis

int16[24] gyro_x          # gyro in the FRD board frame X-axis
int16[24] gyro_y          # gyro in the FRD board frame Y-axis
int16[24] gyro_z          # gyro in the FRD board frame Z-axis

float32 temperature       # temperature in degrees Celsius

uint32 error_count

uint8 ORB_QUEUE_LENGTH = 8
