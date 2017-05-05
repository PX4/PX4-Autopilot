# 使用这个文件可在上位机交叉编译px4firmware
# use this file to cross compile px4  for raspberry pi without any shield
include(configs/posix_rpi_common)

if("$ENV{RPI_USE_CLANG}" STREQUAL "1")
        set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf-raspbian-clang.cmake)
else()
        set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/toolchains/Toolchain-arm-linux-gnueabihf-raspbian.cmake)
endif()

set(CMAKE_PROGRAM_PATH
        "${RPI_TOOLCHAIN_DIR}/gcc-linaro-arm-linux-gnueabihf-raspbian/bin"
        ${CMAKE_PROGRAM_PATH}
)
