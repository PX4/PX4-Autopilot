# - Try to find  MAVLink
# Once done, this will define
#
#  MAVLINK_FOUND        : library found
#  MAVLINK_INCLUDE_DIRS : include directories
#  MAVLINK_VERSION      : version

##
# This file was compied form jsbsim_bridge submodule of PX4/Firmware
##


# macros
include(FindPackageHandleStandardArgs)

# Check for ROS_DISTRO
find_program(ROSVERSION rosversion)
execute_process(COMMAND ${ROSVERSION} -d
    OUTPUT_VARIABLE ROS_DISTRO
    OUTPUT_STRIP_TRAILING_WHITESPACE
)

set(_MAVLINK_EXTRA_SEARCH_HINTS
    ${CMAKE_SOURCE_DIR}/mavlink/
    ../../mavlink/
    ../mavlink/
    ${CATKIN_DEVEL_PREFIX}/
    )

set(_MAVLINK_EXTRA_SEARCH_PATHS
    /usr/
    /usr/local/
    )

# look for in the hints first
find_path(_MAVLINK_INCLUDE_DIR
    NAMES mavlink/v1.0/mavlink_types.h mavlink/v2.0/mavlink_types.h
    PATH_SUFFIXES include
    HINTS ${_MAVLINK_EXTRA_SEARCH_HINTS}
    NO_DEFAULT_PATH
    )

# look for in the hard-coded paths
find_path(_MAVLINK_INCLUDE_DIR
    NAMES mavlink/v1.0/mavlink_types.h mavlink/v2.0/mavlink_types.h
    PATH_SUFFIXES include
    PATHS ${_MAVLINK_EXTRA_SEARCH_PATHS}
    NO_CMAKE_PATH
    NO_CMAKE_ENVIRONMENT_PATH
    NO_SYSTEM_ENVIRONMENT_PATH
    NO_CMAKE_SYSTEM_PATH
    )

# look specifically for the ROS version if no other was found
find_path(_MAVLINK_INCLUDE_DIR
   NAMES mavlink/v1.0/mavlink_types.h mavlink/v2.0/mavlink_types.h
   PATH_SUFFIXES include
   PATHS /opt/ros/${ROS_DISTRO}/
   )

# read the version
if (EXISTS ${_MAVLINK_INCLUDE_DIR}/mavlink/config.h)
    file(READ ${_MAVLINK_INCLUDE_DIR}/mavlink/config.h MAVLINK_CONFIG_FILE)
    string(REGEX MATCH "#define MAVLINK_VERSION[ ]+\"(([0-9]+\\.)+[0-9]+)\""
        _MAVLINK_VERSION_MATCH "${MAVLINK_CONFIG_FILE}")
    set(MAVLINK_VERSION "${CMAKE_MATCH_1}")
else()
    set(MAVLINK_VERSION "2.0")
endif()

# handle arguments
set(MAVLINK_INCLUDE_DIRS ${_MAVLINK_INCLUDE_DIR})
find_package_handle_standard_args(
    MAVLink
    REQUIRED_VARS MAVLINK_INCLUDE_DIRS MAVLINK_VERSION
    VERSION_VAR MAVLINK_VERSION
    )
