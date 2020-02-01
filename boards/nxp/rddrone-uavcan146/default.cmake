px4_add_board(
	PLATFORM nuttx
	VENDOR nxp
	MODEL rddrone-uavcan146
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4

	UAVCAN_INTERFACES 2

	DRIVERS

	MODULES

	SYSTEMCMDS
		i2cdetect

	EXAMPLES
	)
