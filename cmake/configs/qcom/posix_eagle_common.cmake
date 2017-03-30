include(configs/qcom/posix_common)

set(CMAKE_TOOLCHAIN_FILE ${PX4_SOURCE_DIR}/src/lib/DriverFramework/dspal/cmake_hexagon/toolchain/Toolchain-arm-linux-gnueabihf.cmake)

# This definition allows to differentiate if this just the usual POSIX build
# or if it is for the Snapdragon.
add_definitions(
	-D__PX4_POSIX_EAGLE
)
