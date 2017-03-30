include(qurt/px4_impl_qurt)

if ("$ENV{HEXAGON_SDK_ROOT}" STREQUAL "")
	message(FATAL_ERROR "Enviroment variable HEXAGON_SDK_ROOT must be set")
else()
	set(HEXAGON_SDK_ROOT $ENV{HEXAGON_SDK_ROOT})
endif()

set(CONFIG_SHMEM "1")

# Get $QC_SOC_TARGET from environment if existing.
if (DEFINED ENV{QC_SOC_TARGET})
	set(QC_SOC_TARGET $ENV{QC_SOC_TARGET})
else()
	set(QC_SOC_TARGET "APQ8074")
endif()

# Disable the creation of the parameters.xml file by scanning individual
# source files, and scan all source files.  This will create a parameters.xml
# file that contains all possible parameters, even if the associated module
# is not used.  This is necessary for parameter synchronization between the 
# ARM and DSP processors.
set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PX4_SOURCE_DIR}/cmake/cmake_hexagon")
include(toolchain/Toolchain-qurt)
include(qurt_flags)
include_directories(${HEXAGON_SDK_INCLUDES})


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
	platforms/posix/drivers/df_isl29501_wrapper

	#
	# System commands
	#
	systemcmds/param

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/position_estimator_inav
	modules/local_position_estimator
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
	modules/land_detector

	#
	# PX4 drivers
	#
	drivers/gps
	drivers/pwm_out_rc_in
	drivers/spektrum_rc
	drivers/qshell/qurt
	drivers/snapdragon_pwm_out

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
	lib/rc
	lib/version
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

set(config_df_driver_list
	mpu9250
	bmp280
	hmc5883
	trone
	isl29501
	)
