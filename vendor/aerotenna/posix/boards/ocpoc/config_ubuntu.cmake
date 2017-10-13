#set toolchain
set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-gcc-arm-linux-gnueabihf.cmake)

set(CMAKE_PROGRAM_PATH
	${CMAKE_PROGRAM_PATH}
)

include(posix/px4_impl_posix)

add_definitions(
  -D__PX4_POSIX_OCPOC
  -D__DF_LINUX # For DriverFramework
  -D__DF_OCPOC # For DriverFramework
  -D__PX4_POSIX
)

set(config_module_list
	#
	# Board support modules
	#
	drivers/device
	modules/sensors
	platforms/posix/drivers/df_mpu9250_wrapper
	platforms/posix/drivers/df_ms5611_wrapper
	platforms/posix/drivers/df_hmc5883_wrapper

	#
	# System commands
	#
	systemcmds/param
	systemcmds/mixer
	systemcmds/ver
	systemcmds/esc_calib
	systemcmds/reboot
	systemcmds/topic_listener
	systemcmds/perf

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
	modules/sdlog2
	modules/logger
	modules/commander
	modules/systemlib/param
	modules/systemlib
	modules/systemlib/mixer
	modules/uORB
	modules/dataman
	modules/land_detector
	modules/navigator
	modules/mavlink

	#
	# PX4 drivers
	#
	drivers/gps
	drivers/ocpoc_adc
	drivers/ocpoc_sbus_rc_in
	drivers/linux_pwm_out
	drivers/rgbled

	#
	# Libraries
	#
	lib/controllib
	lib/mathlib
	lib/mathlib/math/filter
	lib/geo
	lib/ecl
	lib/geo_lookup
	lib/launchdetection
	lib/external_lgpl
	lib/conversion
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery
	lib/version
	lib/DriverFramework/framework
	lib/rc
	lib/led

	#
	# POSIX
	#
	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
	
	examples/px4_simple_app
)

#
# DriverFramework driver
#
set(config_df_driver_list
	mpu9250
	ms5611
	hmc5883
)
