include(qurt/px4_impl_qurt)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/cmake_hexagon/toolchain/Toolchain-qurt.cmake)
include(${CMAKE_SOURCE_DIR}/cmake/cmake_hexagon/qurt_app.cmake)

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

