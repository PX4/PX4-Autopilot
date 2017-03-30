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
	modules/systemlib/mixer
	modules/uORB

	#
	# Libraries
	#
	lib/mathlib
	lib/mathlib/math/filter
	lib/conversion
	lib/DriverFramework/framework

	#
	# QuRT port
	#
	platforms/common
	platforms/qurt/px4_layer
	platforms/posix/work_queue
	platforms/qurt/tests/hello
	platforms/posix/tests/vcdev_test
	platforms/posix/tests/hrt_test
	platforms/posix/tests/wqueue
	)

