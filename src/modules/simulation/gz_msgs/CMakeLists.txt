############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
############################################################################

# message(FATAL_ERROR "JAKE JAKE JAKE JAKE JAKE JAKE JAKE JAKE JAKE JAKE JAKE JAKE JAKE JAKE JAKE JAKE JAKE JAKE ")

# Check for Gazebo first
if(NOT DEFINED ENV{GZ_DISTRO})
    set(GZ_DISTRO "harmonic" CACHE STRING "Gazebo distribution to use")
else()
    set(GZ_DISTRO $ENV{GZ_DISTRO})
endif()

# Set versions based on distribution
if(GZ_DISTRO STREQUAL "harmonic")
    set(GZ_CMAKE_VERSION "3")
    set(GZ_MSGS_VERSION "10")
    set(GZ_TRANSPORT_VERSION "13")
elseif(GZ_DISTRO STREQUAL "ionic")
    set(GZ_CMAKE_VERSION "4")
    set(GZ_MSGS_VERSION "11")
    set(GZ_TRANSPORT_VERSION "14")
else()
    message(FATAL_ERROR "Unknown Gazebo distribution: ${GZ_DISTRO}")
endif()

# Find required packages
find_package(gz-transport${GZ_TRANSPORT_VERSION})
if(gz-transport${GZ_TRANSPORT_VERSION}_FOUND)
    find_package(gz-cmake${GZ_CMAKE_VERSION} REQUIRED)
    find_package(gz-msgs${GZ_MSGS_VERSION} REQUIRED)
    find_package(Protobuf REQUIRED)

    # Generate protobuf messages
    file(GLOB MSGS_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
    PROTOBUF_GENERATE_CPP(PROTO_SRCS PROTO_HDRS ${MSGS_PROTOS})

    # Create library
    add_library(px4_gz_msgs STATIC
        ${PROTO_SRCS}
        ${PROTO_HDRS}
    )

    target_include_directories(px4_gz_msgs
        PUBLIC
            ${CMAKE_CURRENT_BINARY_DIR}
            ${Protobuf_INCLUDE_DIRS}
    )
    target_link_libraries(px4_gz_msgs PUBLIC ${PROTOBUF_LIBRARIES})

    # Export the binary dir for other modules
    set(PX4_GZ_MSGS_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR} CACHE INTERNAL "")
endif()
