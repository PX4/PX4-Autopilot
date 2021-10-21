include (${CMAKE_CURRENT_LIST_DIR}/uavcan_board_identity)

px4_add_board(
	PLATFORM nuttx
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	CONSTRAINED_FLASH
	ROMFSROOT cannode
	UAVCAN_INTERFACES 1
	DRIVERS
		adc/board_adc
		barometer/bmp388
		bootloaders
		gps
		imu/invensense/icm20649
		lights/rgbled_ncp5623c
		magnetometer/bosch/bmm150
		uavcannode
	MODULES
		#ekf2
		sensors
	SYSTEMCMDS
		#i2cdetect
		param
		#perf
		#top
		#topic_listener
		#uorb
		#ver
		#work_queue
)
