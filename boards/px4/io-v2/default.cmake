
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL io-v2
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m3
	CHIP_MANUFACTURER stm
	CHIP stm32f1
	DRIVERS
		stm32
	MODULES
		px4iofirmware
	)
