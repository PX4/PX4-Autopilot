uint8 TECS_MODE_NORMAL = 0
uint8 TECS_MODE_UNDERSPEED = 1
uint8 TECS_MODE_TAKEOFF = 2
uint8 TECS_MODE_LAND = 3
uint8 TECS_MODE_LAND_THROTTLELIM = 4
uint8 TECS_MODE_BAD_DESCENT = 5
uint8 TECS_MODE_CLIMBOUT = 6

uint64 timestamp		# timestamp, in microseconds since system start */

float32 altitudeSp
float32 altitude_filtered
float32 flightPathAngleSp
float32 flightPathAngle
float32 flightPathAngleFiltered
float32 airspeedSp
float32 airspeed_filtered
float32 airspeedDerivativeSp
float32 airspeedDerivative

float32 totalEnergyError
float32 energyDistributionError
float32 totalEnergyRateError
float32 energyDistributionRateError

float32 throttle_integ
float32 pitch_integ

uint8 mode
