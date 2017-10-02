# 使用这个文件可在上位机交叉编译px4firmware
# use this file to cross compile px4  for raspberry pi without any shield
include(configs/posix_rpi_common)

add_definitions(
	-D__DF_RPI_SINGLE # For raspberry pi without shield accessory
)

set(CMAKE_TOOLCHAIN_FILE toolchains/Toolchain-arm-linux-gnueabihf)

set(CMAKE_PROGRAM_PATH
	"${RPI_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf-raspbian/bin"
	${CMAKE_PROGRAM_PATH}
)
