include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE cmake/toolchains/Toolchain-native.cmake)

set(config_module_list
	drivers/led
	drivers/device
	drivers/boards/sitl
	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
	platforms/posix/drivers/adcsim
	platforms/posix/drivers/gpssim
	platforms/posix/drivers/tonealrmsim
	platforms/posix/drivers/accelsim
	platforms/posix/drivers/airspeedsim
	platforms/posix/drivers/barosim
	platforms/posix/drivers/gyrosim
	systemcmds/param
	systemcmds/mixer
	systemcmds/ver
	systemcmds/esc_calib
	systemcmds/reboot
	systemcmds/topic_listener
	modules/uORB
	modules/systemlib
	modules/systemlib/mixer
	modules/sensors
	modules/simulator
	modules/mavlink
	modules/attitude_estimator_ekf
	modules/attitude_estimator_q
	modules/ekf_att_pos_estimator
	modules/position_estimator_inav
	modules/navigator
	modules/mc_pos_control
	modules/mc_att_control
	modules/mc_pos_control_multiplatform
	modules/mc_att_control_multiplatform
	modules/land_detector
	modules/fw_att_control
	modules/fw_pos_control_l1
	modules/dataman
	modules/sdlog2
	modules/commander
	modules/controllib
	lib/mathlib
	lib/mathlib/math/filter
	lib/conversion
	lib/ecl
	lib/external_lgpl
	lib/geo
	lib/geo_lookup
	lib/launchdetection
	)


set(config_firmware_options
	PARAM_XML # generate param xml
	)

set(config_extra_builtin_cmds
	serdis_main
	sercon_main
	)

add_custom_target(sercon)
set_target_properties(sercon PROPERTIES
	MAIN "sercon" STACK "2048")

add_custom_target(serdis)
set_target_properties(serdis PROPERTIES
	MAIN "serdis" STACK "2048")
