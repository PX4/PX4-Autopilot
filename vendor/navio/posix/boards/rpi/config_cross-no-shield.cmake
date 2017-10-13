# 使用这个文件可在上位机交叉编译px4firmware
# use this file to cross compile px4  for raspberry pi without any shield
include(posix_rpi_common)

add_definitions(
	-D__DF_RPI_SINGLE # For raspberry pi without shield accessory
)

SET(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf.cmake)
