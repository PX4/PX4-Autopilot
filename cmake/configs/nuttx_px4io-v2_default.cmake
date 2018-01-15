
px4_nuttx_configure(HWCLASS m3 CONFIG nsh)

set(config_module_list
	drivers/boards/px4io-v2
	drivers/stm32
	lib/mixer
	lib/rc
	modules/px4iofirmware
)
