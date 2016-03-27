include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/cmake_hexagon/qurt_app.cmake)

set(CONFIG_SHMEM "1")

# This definition allows to differentiate if this just the usual POSIX build
# or if it is for the Snapdragon.
add_definitions(
	-D__PX4_POSIX_EAGLE
	)

set(config_module_list
	drivers/device
	drivers/blinkm
	drivers/pwm_out_sim
	drivers/rgbled
	drivers/led
	drivers/boards/sitl
	drivers/qshell/posix

	systemcmds/param
	systemcmds/mixer
	systemcmds/ver
	systemcmds/topic_listener

	modules/mavlink

	modules/attitude_estimator_ekf
	modules/ekf_att_pos_estimator

	modules/mc_pos_control
	modules/mc_att_control

	modules/param
	modules/systemlib
	modules/systemlib/mixer
	modules/uORB
	modules/muorb/krait
	modules/sensors
	modules/dataman
	modules/sdlog2
	modules/simulator
	modules/commander
	modules/controllib

	lib/mathlib
	lib/mathlib/math/filter
	lib/conversion
	lib/ecl
	lib/geo
	lib/geo_lookup
	lib/terrain_estimation
	lib/runway_takeoff
	lib/tailsitter_recovery

	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
	)
