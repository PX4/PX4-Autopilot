include(posix_rpi_common)

add_definitions(
	-D __DF_RPI
	)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/platforms/posix/cmake/toolchains/Toolchain-native.cmake)
