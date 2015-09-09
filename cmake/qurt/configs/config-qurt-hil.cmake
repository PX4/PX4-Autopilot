function(px4_set_config_modules out_module_list)
	set(config_module_list
		drivers/device
		drivers/boards/sitl
		drivers/pwm_out_sim
		drivers/led
		drivers/rgbled
		modules/sensors
		modules/uORB

	#	drivers/blinkm
	#	drivers/ms5611

	#
	# System commands
	#
		systemcmds/param
		systemcmds/mixer

	#
	# General system control
	#
	#	modules/mavlink

	#
	# Estimation modules (EKF/ SO3 / other filters)
	#
	#	modules/attitude_estimator_ekf
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
		modules/systemlib
		modules/systemlib/mixer
		modules/uORB
	#MODULES		+= modules/dataman
	#MODULES		+= modules/sdlog2
	#MODULES		+= modules/simulator
		modules/commander

	#
	# Libraries
	#
		lib/mathlib
		lib/mathlib/math/filter
		lib/geo
		lib/geo_lookup
		lib/conversion
		modules/controllib

	#
	# QuRT port
	#
		platforms/qurt/px4_layer
		platforms/posix/work_queue
	#	platforms/posix/drivers/accelsim
	#	platforms/posix/drivers/gyrosim
	#	platforms/posix/drivers/adcsim
	#	platforms/posix/drivers/barosim

	#
	# Unit tests
	#
	#	platforms/qurt/tests/muorb
	#	platforms/posix/tests/vcdev_test
	#	platforms/posix/tests/hrt_test
	#	platforms/posix/tests/wqueue

	#
	# sources for muorb over fastrpc
	#
		modules/muorb/adsp
		)
	message(STATUS "modules: ${config_module_list}")
	set(${out_module_list} ${config_module_list} PARENT_SCOPE)
endfunction()

