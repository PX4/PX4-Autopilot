
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL io-v2pro
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	DRIVERS
		stm32
	MODULES
		px4iofirmware
	)
