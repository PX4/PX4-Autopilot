# Excelsior is the code name of a board currently in development.
#
# This cmake config builds for POSIX, so the part of the flight stack running
# on the Linux side of the Snapdragon.
include(configs/posix_sdflight_default)
set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/cmake/cmake_hexagon/toolchain/Toolchain-arm-oemllib32-linux-gnueabi.cmake)

# This definition allows to differentiate the specific board.
add_definitions(
	-D__PX4_POSIX_EXCELSIOR
)
