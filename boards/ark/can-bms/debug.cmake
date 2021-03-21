include (${CMAKE_CURRENT_LIST_DIR}/uavcan_board_identity)

px4_add_board(
	PLATFORM nuttx
	VENDOR ark
	MODEL can-bms
	LABEL debug
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOT cannode
	UAVCAN_INTERFACES 1
	DRIVERS
		bootloaders
		batt_smbus
		uavcannode
	MODULES
		#ekf2
		load_mon
		#sensors
	SYSTEMCMDS
		mft
		mtd
		param
		perf
		reboot
		system_time
		top
		topic_listener
		ver
		work_queue
)
