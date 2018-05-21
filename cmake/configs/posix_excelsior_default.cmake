# Excelsior is the code name of a board currently in development.
#
# This cmake config builds for POSIX, so the part of the flight stack running
# on the Linux side of the Snapdragon.

include(common/px4_git)
px4_add_git_submodule(TARGET git_cmake_hexagon PATH "cmake/cmake_hexagon")

list(APPEND CMAKE_MODULE_PATH "${PX4_SOURCE_DIR}/cmake/cmake_hexagon")

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/cmake_hexagon/toolchain/Toolchain-arm-linux-gnueabihf.cmake)

# This definition allows to differentiate the specific board.
add_definitions(-D__PX4_POSIX_EXCELSIOR)

# Get $QC_SOC_TARGET from environment if existing.
if (DEFINED ENV{QC_SOC_TARGET})
	set(QC_SOC_TARGET $ENV{QC_SOC_TARGET})
else()
	set(QC_SOC_TARGET "APQ8096")
endif()

# TODO: check this
set(DSP_TYPE "ADSP")

# Disable the creation of the parameters.xml file by scanning individual
# source files, and scan all source files.  This will create a parameters.xml
# file that contains all possible parameters, even if the associated module
# is not used.  This is necessary for parameter synchronization between the
# ARM and DSP processors.
set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

set(CONFIG_SHMEM "1")

set(config_module_list
	drivers/blinkm
	drivers/linux_sbus
	drivers/pwm_out_sim
	drivers/rgbled
	drivers/qshell/posix

	#
	# Testing
	#
	#drivers/distance_sensor/sf0x/sf0x_tests
	#drivers/test_ppm
	#lib/controllib/controllib_test
	#lib/rc/rc_tests
	#modules/commander/commander_tests
	#modules/mavlink/mavlink_tests
	#modules/mc_pos_control/mc_pos_control_tests
	#modules/uORB/uORB_tests
	#systemcmds/tests

	#
	# General system control
	#
	modules/camera_feedback
	modules/commander
	modules/events
	#modules/gpio_led
	modules/land_detector
	#modules/load_mon
	modules/mavlink
	modules/navigator
	modules/simulator
	modules/sensors
	#modules/uavcan

	#
	# Estimation modules
	#
	modules/attitude_estimator_q
	modules/ekf2
	modules/local_position_estimator
	modules/position_estimator_inav
	modules/landing_target_estimator
	modules/wind_estimator

	#
	# Vehicle Control
	#
	modules/fw_att_control
	modules/fw_pos_control_l1
	modules/gnd_att_control
	modules/gnd_pos_control
	modules/mc_att_control
	modules/mc_pos_control
	modules/vtol_att_control

	#
	# Logging
	#
	modules/logger
	#modules/sdlog2

	#
	# Library modules
	#
	modules/dataman
	modules/muorb
)
