include(posix/px4_impl_posix)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf-raspbian.cmake)

add_definitions(
  -D__PX4_POSIX_BEBOP
	)

set(CMAKE_PROGRAM_PATH
	"${RPI_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf-raspbian/bin"
	${CMAKE_PROGRAM_PATH}
	)

set(config_module_list
  examples/px4_simple_app

	drivers/device

	#
	# Library modules
	#
	modules/uORB

	#
	# POSIX
	#
	platforms/common
	platforms/posix/px4_layer
	platforms/posix/work_queue
	
  # 
  # libraries
  #
	lib/DriverFramework/framework
)
