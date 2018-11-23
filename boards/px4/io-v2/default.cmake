
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL io-v2
	TOOLCHAIN arm-none-eabi
	PROCESSOR cortex-m3
	DRIVERS
		stm32
	MODULES
		px4iofirmware
	)
