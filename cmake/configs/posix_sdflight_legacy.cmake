include(common/px4_git)
px4_add_git_submodule(TARGET git_cmake_hexagon PATH "cmake/cmake_hexagon")


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PX4_SOURCE_DIR}/cmake/cmake_hexagon")

# Disable the creation of the parameters.xml file by scanning individual
# source files, and scan all source files.  This will create a parameters.xml
# file that contains all possible parameters, even if the associated module
# is not used.  This is necessary for parameter synchronization between the 
# ARM and DSP processors.
set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

# Get $QC_SOC_TARGET from environment if existing.
if (DEFINED ENV{QC_SOC_TARGET})
	set(QC_SOC_TARGET $ENV{QC_SOC_TARGET})
else()
	set(QC_SOC_TARGET "APQ8074")
endif()

set(CONFIG_SHMEM "1")


set(config_module_list
	drivers/blinkm
	drivers/linux_sbus
	drivers/pwm_out_sim
	drivers/rgbled
	drivers/qshell/posix

	systemcmds/param
	systemcmds/led_control
	systemcmds/mixer
	systemcmds/ver
	systemcmds/topic_listener
	systemcmds/tune_control

	modules/mavlink

	modules/attitude_estimator_q
	modules/position_estimator_inav
	modules/local_position_estimator
	modules/landing_target_estimator
	modules/ekf2

	modules/mc_pos_control
	modules/mc_att_control

	modules/muorb/krait
	modules/sensors
	modules/dataman
	modules/sdlog2
	modules/logger
	modules/simulator
	modules/commander
	modules/navigator
	)
