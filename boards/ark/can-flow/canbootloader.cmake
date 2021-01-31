
# UAVCAN boot loadable Module ID
set(uavcanblid_sw_version_major 0)
set(uavcanblid_sw_version_minor 1)
add_definitions(
	-DAPP_VERSION_MAJOR=${uavcanblid_sw_version_major}
	-DAPP_VERSION_MINOR=${uavcanblid_sw_version_minor}
)

set(uavcanblid_hw_version_major 1)
set(uavcanblid_hw_version_minor 0)
set(uavcanblid_name "\"org.ark.can-flow\"")

add_definitions(
	-DHW_UAVCAN_NAME=${uavcanblid_name}
	-DHW_VERSION_MAJOR=${uavcanblid_hw_version_major}
	-DHW_VERSION_MINOR=${uavcanblid_hw_version_minor}
)

px4_add_board(
	PLATFORM nuttx
	VENDOR ark
	MODEL can-flow
	LABEL canbootloader
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	DRIVERS
		bootloaders
)
