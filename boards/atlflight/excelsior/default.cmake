
# Excelsior is the code name of a board currently in development.

include(px4_git)
px4_add_git_submodule(TARGET git_cmake_hexagon PATH "${PX4_SOURCE_DIR}/boards/atlflight/cmake_hexagon")
list(APPEND CMAKE_MODULE_PATH "${PX4_SOURCE_DIR}/boards/atlflight/cmake_hexagon")

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

set(CONFIG_SHMEM "1")
add_definitions(-DORB_COMMUNICATOR)

# This definition allows to differentiate if this just the usual POSIX build
# or if it is for the Snapdragon.
add_definitions(-D__PX4_POSIX_EXCELSIOR)

px4_add_board(
	PLATFORM posix
	VENDOR atlflight
	MODEL excelsior
	LABEL default
	TESTING
	TOOLCHAIN
		toolchain/Toolchain-arm-oemllib32-linux-gnueabi

	DRIVERS
		#barometer # all available barometer drivers
		batt_smbus
		camera_trigger
		differential_pressure # all available differential pressure drivers
		distance_sensor # all available distance sensor drivers
		gps
		linux_sbus
		#imu # all available imu drivers
		#magnetometer # all available magnetometer drivers
		rgbled
		pwm_out_sim
		qshell/posix
		#telemetry # all available telemetry drivers
		vmount

	MODULES
		muorb/krait

		attitude_estimator_q
		camera_feedback
		commander
		dataman
		ekf2
		events
		fw_att_control
		fw_pos_control_l1
		gnd_att_control
		gnd_pos_control
		#gpio_led
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_pos_control
		navigator
		position_estimator_inav
		sensors
		simulator
		#uavcan
		vtol_att_control
		wind_estimator

	SYSTEMCMDS
		#bl_update
		#config
		#dumpfile
		esc_calib
		#hardfault_log
		led_control
		mixer
		motor_ramp
		#mtd
		#nshterm
		param
		perf
		pwm
		reboot
		sd_bench
		shutdown
		tests # tests and test runner
		top
		topic_listener
		tune_control
		ver

	EXAMPLES
		bottle_drop # OBC challenge
		fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		#hwtest # Hardware test
		px4_mavlink_debug # Tutorial code from https://px4.io/dev/debug_values
		px4_simple_app # Tutorial code from https://px4.io/dev/px4_simple_app
		rover_steering_control # Rover example app
		segway
	)
