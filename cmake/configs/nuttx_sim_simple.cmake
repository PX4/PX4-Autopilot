include(nuttx/px4_impl_nuttx)

message(WARNING "this is a work in progress and doesn't build yet")

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)

set(config_module_list
	#platforms/nuttx
	#platforms/nuttx/px4_layer
	platforms/common
	#drivers/led
	drivers/device
	#modules/systemlib
	#modules/uORB
	#examples/px4_simple_app
	#lib/mathlib/math/filter
	#lib/conversion
	)

set(config_extra_builtin_cmds
	)
