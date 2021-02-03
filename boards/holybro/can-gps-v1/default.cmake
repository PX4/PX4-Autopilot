

# UAVCAN boot loadable Module ID
set(uavcanblid_sw_version_major 0)
set(uavcanblid_sw_version_minor 1)
add_definitions(
	-DAPP_VERSION_MAJOR=${uavcanblid_sw_version_major}
	-DAPP_VERSION_MINOR=${uavcanblid_sw_version_minor}
)

set(uavcanblid_hw_version_major 1)
set(uavcanblid_hw_version_minor 0)
set(uavcanblid_name "\"org.holybro.can-gps-v1\"")

add_definitions(
	-DHW_UAVCAN_NAME=${uavcanblid_name}
	-DHW_VERSION_MAJOR=${uavcanblid_hw_version_major}
	-DHW_VERSION_MINOR=${uavcanblid_hw_version_minor}
)

px4_add_board(
	PLATFORM nuttx
	VENDOR holybro
	MODEL can-gps-v1
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOT cannode
	UAVCAN_INTERFACES 2
	DRIVERS
		adc/board_adc
		barometer/bmp388
		bootloaders
		gps
		imu/bosch/bmi088
		lights/rgbled_ncp5623c
		magnetometer/bmm150
		uavcannode
	MODULES
		load_mon
	SYSTEMCMDS
		i2cdetect
		param
		perf
		reboot
		top
		topic_listener
		ver
		work_queue
)
