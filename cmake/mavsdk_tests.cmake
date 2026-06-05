############################################################################
#
# mavsdk_tests ExternalProject
#
# Builds the MAVSDK C++ integration tests as an external project.
# Available to any SITL build (SIH, Gazebo, etc.) when the MAVSDK
# library is installed. Use: make <sitl_target> mavsdk_tests
#
# The ExternalProject is always defined (EXCLUDE_FROM_ALL) so the
# target exists even if MAVSDK is not yet installed. The actual build
# will fail at compile time if MAVSDK is missing, which is the
# expected workflow (install MAVSDK, then build the target).
#
############################################################################

if(TARGET mavsdk_tests)
	return()
endif()

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
