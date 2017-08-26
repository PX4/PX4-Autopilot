include(nuttx/px4_impl_nuttx)

px4_nuttx_configure(HWCLASS m4 CONFIG bootloader)

# Bring in common uavcan hardware identity definitions
include(configs/uavcan_board_ident/s2740vc-v1)

set(config_module_list
	drivers/boards/s2740vc-v1/bootloader
)