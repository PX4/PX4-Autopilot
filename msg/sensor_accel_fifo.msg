uint64 timestamp          # time since system start (microseconds)
uint64 timestamp_sample

uint32 device_id          # unique device ID for the sensor that does not change between power cycles

float32 dt                # delta time between samples (microseconds)
float32 scale

uint8 samples             # number of valid samples

<<<<<<< HEAD
int16[32] x               # acceleration in the NED X board axis in m/s/s
int16[32] y               # acceleration in the NED Y board axis in m/s/s
int16[32] z               # acceleration in the NED Z board axis in m/s/s

uint8 rotation            # Direction the sensor faces (see Rotation enum)
=======
int16[32] x               # acceleration in the FRD body frame X-axis in m/s^2
int16[32] y               # acceleration in the FRD body frame Y-axis in m/s^2
int16[32] z               # acceleration in the FRD body frame Z-axis in m/s^2
>>>>>>> ebdcde2ca02aa89933daecd7a1d0cbb87a33dc00
