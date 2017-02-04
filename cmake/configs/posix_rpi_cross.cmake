include(configs/posix_rpi_common)

if("$ENV{RPI_USE_CLANG}" STREQUAL "1")
	set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf-clang.cmake)
else()
	set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf.cmake)
endif()


set(CMAKE_PROGRAM_PATH
	"${PX4_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf/bin"
	${CMAKE_PROGRAM_PATH}
)
