include(posix/px4_impl_posix)

# Use build stubs unless explicitly set not to
if("${DSPAL_STUBS_ENABLE}" STREQUAL "")
	set(DSPAL_STUBS_ENABLE "1")
endif()

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf.cmake)

set(config_module_list
	drivers/device
	drivers/boards/sitl
	drivers/led

	systemcmds/param
	systemcmds/ver

	modules/mavlink

	modules/param
	modules/systemlib
	modules/uORB
	modules/sensors
	modules/dataman
	modules/sdlog2
	modules/simulator
	modules/commander

	lib/mathlib
	lib/mathlib/math/filter
	lib/geo
	lib/geo_lookup
	lib/conversion

	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
	modules/muorb/krait
	)

