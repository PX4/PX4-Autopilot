uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles
bool is_external          # true if the sensor is not mounted on the flight controller board itself

float32 dt                # delta time between samples (microseconds)
float32 scale             # m/s^2 per count

uint8 samples             # number of valid samples

int16[32] x               # FRD board frame X-axis raw counts (m/s^2 = x * scale)
int16[32] y               # FRD board frame Y-axis raw counts (m/s^2 = y * scale)
int16[32] z               # FRD board frame Z-axis raw counts (m/s^2 = z * scale)
