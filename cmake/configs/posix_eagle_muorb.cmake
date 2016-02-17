include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE cmake/toolchains/Toolchain-arm-linux-gnueabihf.cmake)

set(config_module_list
	drivers/device

	modules/uORB

	platforms/posix/px4_layer
	platforms/posix/work_queue

	modules/muorb/krait
	)

