include(qurt/px4_impl_qurt)

if ("$ENV{HEXAGON_SDK_ROOT}" STREQUAL "")
	message(FATAL_ERROR "Enviroment variable HEXAGON_SDK_ROOT must be set")
else()
	set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})
endif()

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/cmake_hexagon/toolchain/Toolchain-qurt.cmake)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/cmake_hexagon")

set(config_generate_parameters_scope ALL)

set(config_module_list
	drivers/device
	drivers/boards/sitl
	drivers/pwm_out_sim
	drivers/led
	drivers/rgbled
	modules/sensors

	#
	# System commands
	#
	systemcmds/param
	systemcmds/mixer

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	#modules/attitude_estimator_ekf
	modules/ekf_att_pos_estimator
	modules/attitude_estimator_q
	modules/position_estimator_inav

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

	#
	# Libraries
	#
	lib/mathlib
	lib/mathlib/math/filter
	lib/geo
	lib/geo_lookup
	lib/conversion
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery
	lib/controllib
	lib/DriverFramework/framework

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
