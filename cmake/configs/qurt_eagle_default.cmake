include(qurt/px4_impl_qurt)

#if ("${HEXAGON_DRIVERS_ROOT}" #STREQUAL "")
#	message(FATAL_ERROR "HEXAGON_DRIVERS_ROOT is not set")
#endif()

#if ("${EAGLE_DRIVERS_SRC}" STREQUAL "")
#	message(FATAL_ERROR "EAGLE_DRIVERS_SRC is not set")
#endif()

#include_directories(${HEXAGON_DRIVERS_ROOT}/inc)

set(CONFIG_SHMEM "1")

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/cmake_hexagon/toolchain/Toolchain-qurt.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/cmake_hexagon/qurt_app.cmake)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	modules/sensors
	platforms/posix/drivers/df_mpu9250_wrapper
	platforms/posix/drivers/df_bmp280_wrapper
	platforms/posix/drivers/df_hmc5883_wrapper
	platforms/posix/drivers/df_trone_wrapper

	#
	# System commands
	#
	systemcmds/param

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	#modules/attitude_estimator_ekf
	modules/ekf_att_pos_estimator
	modules/attitude_estimator_q
	modules/position_estimator_inav
	modules/ekf2

	#
	# Vehicle Control
	#
	modules/mc_att_control
	modules/mc_pos_control

	#
	# Library modules
	#
	modules/param
	modules/systemlib
	modules/systemlib/mixer
	modules/uORB
	modules/commander
	modules/controllib
	modules/land_detector

	#
	# PX4 drivers
	#
	drivers/gps
	drivers/uart_esc
	drivers/qshell/qurt

	#
	# Libraries
	#
	lib/mathlib
	lib/mathlib/math/filter
	lib/geo
	lib/ecl
	lib/geo_lookup
	lib/conversion
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery

	#
	# QuRT port
	#
	platforms/common
	platforms/qurt/px4_layer
	platforms/posix/work_queue

	#
	# sources for muorb over fastrpc
	#
	modules/muorb/adsp
	)

set(config_df_driver_list
	mpu9250
	bmp280
	hmc5883
	trone
	)
