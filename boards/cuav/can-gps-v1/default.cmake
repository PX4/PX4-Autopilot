

# UAVCAN boot loadable Module ID
set(uavcanblid_sw_version_major 0)
set(uavcanblid_sw_version_minor 1)
add_definitions(
	-DAPP_VERSION_MAJOR=${uavcanblid_sw_version_major}
	-DAPP_VERSION_MINOR=${uavcanblid_sw_version_minor}
)

set(uavcanblid_hw_version_major 1)
set(uavcanblid_hw_version_minor 0)
set(uavcanblid_name "\"org.cuav.can-gps-v1\"")

add_definitions(
	-DHW_UAVCAN_NAME=${uavcanblid_name}
	-DHW_VERSION_MAJOR=${uavcanblid_hw_version_major}
	-DHW_VERSION_MINOR=${uavcanblid_hw_version_minor}
)
add_definitions(-DUSE_S_RGB_LED_DMA)

px4_add_board(
	PLATFORM nuttx
	VENDOR cuav
	MODEL can-gps-v1
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	CONSTRAINED_MEMORY
	ROMFSROOT cannode
	UAVCAN_INTERFACES 1
	DRIVERS
		barometer/ms5611
		bootloaders
		gps
		lights/neopixel
		magnetometer/rm3100
		safety_button
		tone_alarm
		uavcannode
	MODULES
		load_mon
	SYSTEMCMDS
		i2cdetect
		led_control
		param
		perf
		reboot
		top
		topic_listener
		tune_control
		ver
		work_queue
)
