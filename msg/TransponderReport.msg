# Transponder report
#
# ADSB report closely matching MAVLink's ADSB_VEHICLE (246) message with few internal extra fields at the end.
# Populated by ADSB receivers, processed for user messaging and navigator, logging and republishing ADSB information.

uint64 timestamp # [us] Time since system start

uint32 icao_address # [-] ICAO address
float64 lat # [deg] Latitude, validity flag: PX4_ADSB_FLAGS_VALID_COORDS
float64 lon # [deg] Longitude, validity flag: PX4_ADSB_FLAGS_VALID_COORDS
uint8 altitude_type # Type from ADSB_ALTITUDE_TYPE enum
float32 altitude # [m] Altitude (ASL), validity flag: PX4_ADSB_FLAGS_VALID_ALTITUDE
float32 heading # [rad] Course over ground, 0 to 2pi, 0 is north, validity flag: PX4_ADSB_FLAGS_VALID_HEADING
float32 hor_velocity # [m/s] Horizontal velocity, validity flag: PX4_ADSB_FLAGS_VALID_VELOCITY
float32 ver_velocity # [m/s] Vertical velocity, positive is up, validity flag: PX4_ADSB_FLAGS_VALID_VELOCITY
char[9] callsign # The callsign, 8+null, validity flag: PX4_ADSB_FLAGS_VALID_CALLSIGN

uint8 emitter_type # [@enum ADSB_EMITTER_TYPE] Type matching MAVLink's ADSB_EMITTER_TYPE enum
uint16 ADSB_EMITTER_TYPE_NO_INFO=0
uint16 ADSB_EMITTER_TYPE_LIGHT=1
uint16 ADSB_EMITTER_TYPE_SMALL=2
uint16 ADSB_EMITTER_TYPE_LARGE=3
uint16 ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE=4
uint16 ADSB_EMITTER_TYPE_HEAVY=5
uint16 ADSB_EMITTER_TYPE_HIGHLY_MANUV=6
uint16 ADSB_EMITTER_TYPE_ROTOCRAFT=7
uint16 ADSB_EMITTER_TYPE_UNASSIGNED=8
uint16 ADSB_EMITTER_TYPE_GLIDER=9
uint16 ADSB_EMITTER_TYPE_LIGHTER_AIR=10
uint16 ADSB_EMITTER_TYPE_PARACHUTE=11
uint16 ADSB_EMITTER_TYPE_ULTRA_LIGHT=12
uint16 ADSB_EMITTER_TYPE_UNASSIGNED2=13
uint16 ADSB_EMITTER_TYPE_UAV=14
uint16 ADSB_EMITTER_TYPE_SPACE=15
uint16 ADSB_EMITTER_TYPE_UNASSGINED3=16
uint16 ADSB_EMITTER_TYPE_EMERGENCY_SURFACE=17
uint16 ADSB_EMITTER_TYPE_SERVICE_SURFACE=18
uint16 ADSB_EMITTER_TYPE_POINT_OBSTACLE=19
uint16 ADSB_EMITTER_TYPE_ENUM_END=20

uint8 tslc # [s] Time since last communication

uint16 flags # [@enum PX4_ADSB_FLAGS] Flags matching MAVLink's ADSB_FLAGS bitmask
uint16 PX4_ADSB_FLAGS_VALID_COORDS = 1
uint16 PX4_ADSB_FLAGS_VALID_ALTITUDE = 2
uint16 PX4_ADSB_FLAGS_VALID_HEADING = 4
uint16 PX4_ADSB_FLAGS_VALID_VELOCITY = 8
uint16 PX4_ADSB_FLAGS_VALID_CALLSIGN = 16
uint16 PX4_ADSB_FLAGS_VALID_SQUAWK = 32
uint16 PX4_ADSB_FLAGS_SIMULATED = 64
uint16 PX4_ADSB_FLAGS_VERTICAL_VELOCITY_VALID = 128
uint16 PX4_ADSB_FLAGS_BARO_VALID = 256
uint16 PX4_ADSB_FLAGS_SOURCE_UAT = 32768

uint16 squawk # [-] Squawk code, validity flag: PX4_ADSB_FLAGS_VALID_SQUAWK

uint8[18] uas_id # [-] Unique UAS ID, not part of ADSB_VEHICLE MAVLink message

uint8 ORB_QUEUE_LENGTH = 16
