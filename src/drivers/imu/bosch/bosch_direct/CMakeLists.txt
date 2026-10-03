set(bmi_sources
	bosch_direct_main.cpp
	BoschDirect.cpp
	BoschDirectModel.cpp
)
if(CONFIG_BOSCH_DIRECT_BMI270)
	list(APPEND bmi_sources Bmi270Config.cpp)
endif()

px4_add_module(
	MODULE drivers__imu__bosch__bosch_direct
	MAIN bosch_direct
	SRCS ${bmi_sources}
	DEPENDS
		px4_work_queue
		drivers_accelerometer
		drivers_gyroscope
)
