include(configs/posix_bbblue_common)

add_definitions(
	 -D __DF_BBBLUE
	)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-native.cmake)
