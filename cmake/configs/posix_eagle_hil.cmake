
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PX4_SOURCE_DIR}/cmake/cmake_hexagon")

# Use build stubs unless explicitly set not to
if("${DSPAL_STUBS_ENABLE}" STREQUAL "")
	set(DSPAL_STUBS_ENABLE "1")
endif()

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/cmake_hexagon/toolchain/Toolchain-arm-linux-gnueabihf.cmake)

set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

# Get $QC_SOC_TARGET from environment if existing.
if (DEFINED ENV{QC_SOC_TARGET})
	set(QC_SOC_TARGET $ENV{QC_SOC_TARGET})
else()
	set(QC_SOC_TARGET "APQ8074")
endif()

set(config_module_list
	drivers/linux_sbus

	systemcmds/param
	systemcmds/ver

	modules/mavlink

	modules/sensors
	modules/dataman
	modules/sdlog2
	modules/logger
	modules/simulator
	modules/commander

	modules/muorb/krait
	)

