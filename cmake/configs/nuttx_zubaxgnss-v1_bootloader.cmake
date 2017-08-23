include(nuttx/px4_impl_nuttx)

px4_nuttx_configure(HWCLASS m3 CONFIG bootloader)

# Bring in common uavcan hardware identity definitions
include(configs/uavcan_board_ident/zubaxgnss-v1)

set(config_module_list
	drivers/boards/zubaxgnss-v1/bootloader
)