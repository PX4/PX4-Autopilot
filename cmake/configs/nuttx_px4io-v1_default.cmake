include(nuttx/px4_impl_nuttx)

px4_nuttx_configure(HWCLASS m3 CONFIG nsh)

set(config_module_list
	drivers/boards
	drivers/stm32
	lib/mixer
	lib/rc
	modules/px4iofirmware
	platforms/common
)
