include(unittests/px4_impl_unittests)

set(CMAKE_TOOLCHAIN_FILE cmake/toolchains/Toolchain-native.cmake)

set(config_module_list
	drivers/device
	drivers/boards/sitl
	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
	modules/uORB
	modules/param
	modules/systemlib
	modules/systemlib/mixer
	examples/unittest
	)


set(config_firmware_options
	PARAM_XML # generate param xml
	)
