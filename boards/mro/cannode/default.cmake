include (${CMAKE_CURRENT_LIST_DIR}/uavcan_board_identity)

px4_add_board(
	PLATFORM nuttx
	VENDOR mro
	MODEL cannode
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_FLASH
	CONSTRAINED_MEMORY
	ROMFSROOT cannode
	UAVCAN_INTERFACES 1
	UAVCAN_TIMER_OVERRIDE 3
	DRIVERS
		#barometer/dps310
		bootloaders
		#gps
		#lights/rgbled_ncp5623c
		#magnetometer/rm3100
		distance_sensor/vl53l1x
		uavcannode
	MODULES
	SYSTEMCMDS
		param
)
