include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/cmake_hexagon/toolchain/Toolchain-arm-linux-gnueabihf.cmake)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PX4_SOURCE_DIR}/cmake/cmake_hexagon")

set(DISABLE_PARAMS_MODULE_SCOPING TRUE)

# Get $QC_SOC_TARGET from environment if existing.
if (DEFINED ENV{QC_SOC_TARGET})
	set(QC_SOC_TARGET $ENV{QC_SOC_TARGET})
else()
	set(QC_SOC_TARGET "APQ8074")
endif()

set(config_module_list
	drivers/device

	modules/uORB

	lib/DriverFramework/framework

	platforms/posix/px4_layer
	platforms/posix/work_queue

	modules/muorb/krait
	)

