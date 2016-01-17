include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)

set(config_module_list
	drivers/device

	systemcmds/param
	systemcmds/ver

	modules/mavlink

	modules/param
	modules/systemlib
	modules/uORB
	modules/dataman

	lib/mathlib
	lib/mathlib/math/filter
	lib/geo
	lib/geo_lookup
	lib/conversion

	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue

)
