include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE cmake/toolchains/Toolchain-arm-linux-gnueabihf.cmake)

set(config_module_list
	drivers/device
	drivers/boards/sitl
	drivers/led

	systemcmds/param
	systemcmds/ver

	modules/mavlink

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

	platforms/posix/px4_layer
	platforms/posix/work_queue
	modules/muorb/krait
	)

