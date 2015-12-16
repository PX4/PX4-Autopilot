include(qurt/px4_impl_qurt)

if ("${HEXAGON_DRIVERS_ROOT}" STREQUAL "")
	message(FATAL_ERROR "HEXAGON_DRIVERS_ROOT is not set")
endif()

if ("${EAGLE_DRIVERS_SRC}" STREQUAL "")
	message(FATAL_ERROR "EAGLE_DRIVERS_SRC is not set")
endif()

include_directories(${HEXAGON_DRIVERS_ROOT}/inc)

# For Actual flight we need to link against the driver dynamic libraries
set(target_libraries
	-L${HEXAGON_DRIVERS_ROOT}/libs 
	mpu9x50
	uart_esc
	csr_gps
	rc_receiver
	)


set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-hexagon-7.2.10.cmake)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	modules/sensors
	$(EAGLE_DRIVERS_SRC)/mpu9x50
	$(EAGLE_DRIVERS_SRC)/uart_esc
	$(EAGLE_DRIVERS_SRC)/rc_receiver
	$(EAGLE_DRIVERS_SRC)/csr_gps

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
