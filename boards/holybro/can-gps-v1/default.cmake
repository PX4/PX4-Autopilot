include (${CMAKE_CURRENT_LIST_DIR}/uavcan_board_identity)

px4_add_board(
	PLATFORM nuttx
	VENDOR holybro
	MODEL can-gps-v1
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOT cannode
	UAVCAN_INTERFACES 2
	DRIVERS
		adc/board_adc
		barometer/bmp388
		bootloaders
		gps
		imu/bosch/bmi088
		lights/rgbled_ncp5623c
		magnetometer/bosch/bmm150
		uavcannode
	MODULES
		load_mon
	SYSTEMCMDS
		i2cdetect
		param
		perf
		reboot
		top
		topic_listener
		ver
		work_queue
)
