uint64 timestamp				# time since system start (microseconds)
int32 id						# Source node ID

uint8[4] unit_type				# Units: mV, mA, Kelvin*10
uint16[4] values 				# mV, mA, Kelvin*10

uint8 UNITS_NONE = 0
uint8 UNITS_MV 	 = 1
uint8 UNITS_MA 	 = 2
uint8 UNITS_CK 	 = 3