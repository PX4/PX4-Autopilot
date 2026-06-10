# MAVSDK integration tests.
#
# Built as an external project so they can link against the system-installed
# MAVSDK. The tests are simulator-agnostic (they drive PX4 over MAVLink) and do
# NOT depend on Gazebo - the simulation backend (Gazebo Classic, SIH, ...) is
# selected at runtime by the test runner, see test/mavsdk_tests/configs/.
include(ExternalProject)
ExternalProject_Add(mavsdk_tests
	SOURCE_DIR ${PX4_SOURCE_DIR}/test/mavsdk_tests
	CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
	BINARY_DIR ${PX4_BINARY_DIR}/mavsdk_tests
	INSTALL_COMMAND ""
	USES_TERMINAL_CONFIGURE true
	USES_TERMINAL_BUILD true
	EXCLUDE_FROM_ALL true
	BUILD_ALWAYS 1
)
