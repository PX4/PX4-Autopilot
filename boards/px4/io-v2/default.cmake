
px4_add_board(
	OS nuttx
	VENDOR px4
	MODEL io-v2
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m3
	DRIVERS
		stm32
	MODULES
		px4iofirmware
	)
