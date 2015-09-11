include(posix/px4_impl_posix)

set(USE_TOOLCHAIN Toolchain-arm-linux-gnueabihf)

function(px4_get_config)

	px4_parse_function_args(
		NAME px4_set_config_modules
		ONE_VALUE OUT_MODULES
		REQUIRED OUT_MODULES
		ARGN ${ARGN})

	set(config_module_list
		drivers/device
		drivers/blinkm
		drivers/pwm_out_sim
		drivers/rgbled
		drivers/led
		modules/sensors

		systemcmds/param
		systemcmds/mixer
		systemcmds/ver

		modules/mavlink

		modules/attitude_estimator_ekf
		modules/ekf_att_pos_estimator

		modules/mc_pos_control
		modules/mc_att_control

		modules/systemlib
		modules/systemlib/mixer
		modules/uORB
		modules/sensors
		modules/dataman
		modules/sdlog2
		modules/simulator
		modules/commander
		modules/controllib

		lib/mathlib
		lib/mathlib/math/filter
		lib/geo
		lib/geo_lookup
		lib/conversion

		platforms/posix/px4_layer
		platforms/posix/work_queue
		)

	set(${OUT_MODULES} ${config_module_list} PARENT_SCOPE)

endfunction()

