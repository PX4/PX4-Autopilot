

# UAVCAN boot loadable Module ID
set(uavcanblid_sw_version_major 0)
set(uavcanblid_sw_version_minor 1)
add_definitions(
	-DAPP_VERSION_MAJOR=${uavcanblid_sw_version_major}
	-DAPP_VERSION_MINOR=${uavcanblid_sw_version_minor}
)

set(uavcanblid_hw_version_major 1)
set(uavcanblid_hw_version_minor 0)
set(uavcanblid_name "\"org.px4.fmu-v4_cannode\"")

add_definitions(
	-DHW_UAVCAN_NAME=${uavcanblid_name}
	-DHW_VERSION_MAJOR=${uavcanblid_hw_version_major}
	-DHW_VERSION_MINOR=${uavcanblid_hw_version_minor}
)

px4_add_board(
	PLATFORM nuttx
	VENDOR st
	MODEL nucleo-f412zg
	LABEL cannode
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT cannode
	UAVCAN_INTERFACES 1
	SERIAL_PORTS
		GPS1:/dev/ttyS1
		TEL1:/dev/ttyS2
	DRIVERS
		adc
		bootloaders
		differential_pressure
		gps
		uavcannode
	MODULES
		sensors
	SYSTEMCMDS
		i2cdetect
		mtd
		param
		perf
		reboot
		top
		ver
		work_queue
)

include(px4_make_uavcan_bootloader)
px4_make_uavcan_bootloadable(
	BOARD ${PX4_BOARD}
	BIN ${PX4_BINARY_DIR}/${PX4_BOARD}.bin
	HWNAME ${uavcanblid_name}
	HW_MAJOR ${uavcanblid_hw_version_major}
	HW_MINOR ${uavcanblid_hw_version_minor}
	SW_MAJOR ${uavcanblid_sw_version_major}
	SW_MINOR ${uavcanblid_sw_version_minor}
)
