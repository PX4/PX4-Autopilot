include (${CMAKE_CURRENT_LIST_DIR}/uavcan_board_identity)

px4_add_board(
	PLATFORM nuttx
	VENDOR freefly
	MODEL can-rtk-gps
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	#CONSTRAINED_FLASH
	ROMFSROOT cannode
	UAVCAN_INTERFACES 1
	DRIVERS
		barometer/bmp388
		bootloaders
		gps
		lights/rgbled_ncp5623c
		magnetometer/isentek/ist8310
		imu/st/lsm9ds1
		uavcannode
	MODULES
		#ekf2
		load_mon
		sensors
	SYSTEMCMDS
		led_control
		mft
		mtd
		param
		perf
		reboot
		system_time
		top
#		topic_listener
		ver
		work_queue
)
