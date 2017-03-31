# This cmake config builds for POSIX, so the part of the flight stack running
# on the Linux side of the Snapdragon.
include(configs/qcom/posix_common)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/src/lib/Driverframework/dspal/cmake_hexagon/toolchain/Toolchain-arm-oemllib32-linux-gnueabi.cmake)

# This definition allows to differentiate the specific board.
add_definitions(
	-D__PX4_POSIX_EXCELSIOR
)
