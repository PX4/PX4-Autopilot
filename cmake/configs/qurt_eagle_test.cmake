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
	modules/systemlib/mixer
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
	platforms/posix/tests/vcdev_test
	platforms/posix/tests/hrt_test
	platforms/posix/tests/wqueue
	)

