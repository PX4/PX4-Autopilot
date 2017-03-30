include(configs/qcom/posix_eagle_common)

# Use build stubs unless explicitly set not to
if("${DSPAL_STUBS_ENABLE}" STREQUAL "")
	set(DSPAL_STUBS_ENABLE "1")
endif()

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
	modules/logger
	modules/simulator
	modules/commander

	lib/controllib
	lib/mathlib
	lib/mathlib/math/filter
	lib/ecl
	lib/geo
	lib/geo_lookup
	lib/conversion
	lib/version
	lib/DriverFramework/framework

	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
	modules/muorb/krait
	)

