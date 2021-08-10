
px4_add_board(
	PLATFORM nuttx
	TOOLCHAIN arm-none-eabi
	CONSTRAINED_FLASH
	ARCHITECTURE cortex-m3
	DRIVERS
	MODULES
		px4iofirmware
	)
