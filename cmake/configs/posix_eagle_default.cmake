include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf.cmake)

set(config_module_list
	drivers/device
	drivers/blinkm
	drivers/pwm_out_sim
	drivers/rgbled
	drivers/led
	drivers/boards/sitl

	systemcmds/param
	systemcmds/mixer
	systemcmds/ver

	modules/mavlink

	modules/attitude_estimator_ekf
	modules/ekf_att_pos_estimator

	modules/mc_pos_control
	modules/mc_att_control

	modules/param
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
	lib/terrain_estimation
	lib/runway_takeoff

	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
	)

