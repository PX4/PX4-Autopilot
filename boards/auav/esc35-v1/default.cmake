
add_definitions(
	-DFLASH_BASED_PARAMS
	-DPARAM_NO_ORB
	-DPARAM_NO_AUTOSAVE
	-DPARAMETER_BUFFER_SIZE=1024
)

# UAVCAN boot loadable Module ID
set(uavcanblid_sw_version_major 0)
set(uavcanblid_sw_version_minor 1)
add_definitions(
	-DAPP_VERSION_MAJOR=${uavcanblid_sw_version_major}
	-DAPP_VERSION_MINOR=${uavcanblid_sw_version_minor}
	)

# Bring in common uavcan hardware identity definitions
include(px4_git)
px4_add_git_submodule(TARGET git_uavcan_board_ident PATH "cmake/configs/uavcan_board_ident")
include(configs/uavcan_board_ident/esc35-v1)

add_definitions(
	-DHW_UAVCAN_NAME=${uavcanblid_name}
	-DHW_VERSION_MAJOR=${uavcanblid_hw_version_major}
	-DHW_VERSION_MINOR=${uavcanblid_hw_version_minor}
)

include(px4_make_uavcan_bootloader)
px4_make_uavcan_bootloadable(
	BOARD auav_esc35-v1
	BIN ${PX4_BINARY_DIR}/auav_esc35-v1.bin
	HWNAME ${uavcanblid_name}
	HW_MAJOR ${uavcanblid_hw_version_major}
	HW_MINOR ${uavcanblid_hw_version_minor}
	SW_MAJOR ${uavcanblid_sw_version_major}
	SW_MINOR ${uavcanblid_sw_version_minor}
)

px4_add_board(
	PLATFORM nuttx
	VENDOR auav
	MODEL esc35-v1
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4

	DRIVERS
		bootloaders
		stm32
		uavcanesc

	MODULES

	SYSTEMCMDS
		config
		reboot
		param
		top
		ver

	)
