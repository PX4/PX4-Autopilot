include(configs/qcom/qurt_eagle_common)

set(config_module_list
	drivers/device

	#
	# System commands
	#
	systemcmds/param

	#
	# Library modules
	#
	modules/param
	modules/systemlib
	modules/uORB

	#
	# QuRT port
	#
	platforms/common
	platforms/qurt/px4_layer
	platforms/posix/work_queue
	platforms/qurt/tests/hello
	)

