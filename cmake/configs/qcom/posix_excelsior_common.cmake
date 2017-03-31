# Excelsior is the code name of a board currently in development

include(configs/qcom/posix_common)

# Use arm-oemllib32-linux-gnueabi toolchain from cmake_hexagon
set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/src/lib/Driverframework/dspal/cmake_hexagon/toolchain/Toolchain-arm-oemllib32-linux-gnueabi.cmake)

# This definition allows to differentiate the specific board
add_definitions(
	-D__PX4_POSIX_EXCELSIOR
)
