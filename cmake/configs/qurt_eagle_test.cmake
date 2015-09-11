include(qurt/px4_impl_qurt)

set(CMAKE_TOOLCHAIN_FILE cmake/toolchains/Toolchain-hexagon.cmake)

set(config_module_list
	drivers/device

	#
	# System commands
	#
	systemcmds/param

	#
	# Library modules
	#
	modules/systemlib
	modules/mixer
	modules/uORB

	#
	# Libraries
	#
	lib/mathlib
	lib/mathlib/math/filter
	lib/conversion

	#
	# QuRT port
	#
	platforms/common
	platforms/qurt/px4_layer
	platforms/posix/work_queue
	platforms/qurt/tests/hello
	platforms/qurt/tests/vcdev_test
	platforms/qurt/tests/hrt_test
	platforms/qurt/tests/wqueue
	)

