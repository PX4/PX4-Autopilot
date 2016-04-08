include(qurt/px4_impl_qurt)

if ("$ENV{HEXAGON_SDK_ROOT}" STREQUAL "")
	message(FATAL_ERROR "Enviroment variable HEXAGON_SDK_ROOT must be set")
else()
	set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})
endif()

if ("$ENV{EAGLE_DRIVERS_SRC}" STREQUAL "")
	message(FATAL_ERROR "Environment variable EAGLE_DRIVERS_SRC must be set")
else()
	set(EAGLE_DRIVERS_SRC $ENV{EAGLE_DRIVERS_SRC})
endif()

STRING(REGEX REPLACE "//" "/" EAGLE_DRIVERS_SRC ${EAGLE_DRIVERS_SRC})
STRING(REGEX REPLACE "/" "__" EAGLE_DRIVERS_MODULE_PREFIX ${EAGLE_DRIVERS_SRC})

#include_directories(${EAGLE_ADDON_ROOT}/flight_controller/hexagon/inc)
include_directories(
	${HEXAGON_SDK_ROOT}/inc
	${HEXAGON_SDK_ROOT}/inc/stddef
	${HEXAGON_SDK_ROOT}/lib/common/qurt/ADSPv5MP/include
	)

message("hexagon_sdk_root is ${HEXAGON_SDK_ROOT}")

set(QURT_ENABLE_STUBS "0")

set(CONFIG_SHMEM "1")

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/cmake_hexagon/toolchain/Toolchain-qurt.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/cmake_hexagon/qurt_app.cmake)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	modules/sensors
	${EAGLE_DRIVERS_SRC}/mpu_spi
	${EAGLE_DRIVERS_SRC}/uart_esc
	${EAGLE_DRIVERS_SRC}/rc_receiver
	${EAGLE_DRIVERS_SRC}/csr_gps

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
	modules/local_position_estimator

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
	lib/controllib
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
