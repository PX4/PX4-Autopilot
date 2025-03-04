uint64 timestamp					# time since system start (microseconds)
uint8 CONNECTED_SERVO_MAX = 8				# The number of SERVOs supported via UAVCAN

uint16 counter  					# incremented by the writing thread everytime new data is stored

DeviceTemperature[8] servo
