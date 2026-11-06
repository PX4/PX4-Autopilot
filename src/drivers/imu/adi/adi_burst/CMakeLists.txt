set(crc_source)
if(CONFIG_ADI_BURST_ADIS16497)
	list(APPEND crc_source AdiBurstDecoder.cpp)
endif()

px4_add_module(
	MODULE drivers__imu__adi__adi_burst
	MAIN adi_burst
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
	SRCS
		adi_burst_main.cpp
		AdiBurst.cpp
		${crc_source}
	DEPENDS
		px4_work_queue
		drivers_accelerometer
		drivers_gyroscope
)
