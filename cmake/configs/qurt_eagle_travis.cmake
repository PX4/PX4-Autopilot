include(configs/qcom/qurt_eagle_common)

# Run a full link with build stubs to make sure qurt target isn't broken
set(QURT_ENABLE_STUBS "1")

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
	systemcmds/led
	systemcmds/mixer

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

	#
	# Libraries
	#
	lib/controllib
	lib/mathlib
	lib/mathlib/math/filter
	lib/geo
	lib/geo_lookup
	lib/conversion
	lib/ecl
	lib/led
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery
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

