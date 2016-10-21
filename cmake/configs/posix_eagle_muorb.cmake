include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf.cmake)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PX4_SOURCE_DIR}/cmake/cmake_hexagon")

set(config_generate_parameters_scope ALL)

set(config_module_list
	drivers/device

	modules/uORB

	lib/DriverFramework/framework

	platforms/posix/px4_layer
	platforms/posix/work_queue

	modules/muorb/krait
	)

