# The Eagle board is the first generation Snapdragon Flight board by Qualcomm

include(configs/qcom/posix_common)

# Use arm-linux-gnueabihf toolchain from cmake_hexagon
set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/src/lib/DriverFramework/dspal/cmake_hexagon/toolchain/Toolchain-arm-linux-gnueabihf.cmake)

# This definition allows to differentiate the specific board
add_definitions(
	-D__PX4_POSIX_EAGLE
)
