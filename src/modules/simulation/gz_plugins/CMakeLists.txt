############################################################################
#
#   Copyright (c) 2025 PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in
#    the documentation and/or other materials provided with the
#    distribution.
# 3. Neither the name PX4 nor the names of its contributors may be
#    used to endorse or promote products derived from this software
#    without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
# OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
# AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
############################################################################

project(px4_gz_plugins)

if(NOT DEFINED ENV{GZ_DISTRO})
    set(GZ_DISTRO "harmonic" CACHE STRING "Gazebo distribution to use")
else()
    set(GZ_DISTRO $ENV{GZ_DISTRO})
endif()

# Define library version combinations for different Gazebo distributions
# https://github.com/gazebo-tooling/gazebodistro/blob/master/collection-harmonic.yaml
if(GZ_DISTRO STREQUAL "harmonic")
    set(GZ_CMAKE_VERSION "3")
    set(GZ_MSGS_VERSION "10")
    set(GZ_TRANSPORT_VERSION "13")
    set(GZ_PLUGIN_VERSION "2")
    set(GZ_SIM_VERSION "8")
    set(GZ_SENSORS_VERSION "8")
    message(STATUS "Using Gazebo Harmonic (cmake:${GZ_CMAKE_VERSION}, msgs:${GZ_MSGS_VERSION}, transport:${GZ_TRANSPORT_VERSION})")
elseif(GZ_DISTRO STREQUAL "ionic")
    set(GZ_CMAKE_VERSION "4")
    set(GZ_MSGS_VERSION "11")
    set(GZ_TRANSPORT_VERSION "14")
    set(GZ_PLUGIN_VERSION "3")
    set(GZ_SIM_VERSION "9")
    set(GZ_SENSORS_VERSION "9")
    message(STATUS "Using Gazebo Ionic (cmake:${GZ_CMAKE_VERSION}, msgs:${GZ_MSGS_VERSION}, transport:${GZ_TRANSPORT_VERSION})")
else()
    message(FATAL_ERROR "Unknown Gazebo distribution: ${GZ_DISTRO}. Valid options are: harmonic or ionic")
endif()

# Use gz-transport as litmus test for presence of gz
find_package(gz-transport${GZ_TRANSPORT_VERSION})

if (gz-transport${GZ_TRANSPORT_VERSION}_FOUND)
    find_package(gz-cmake${GZ_CMAKE_VERSION} REQUIRED)
    find_package(gz-msgs${GZ_MSGS_VERSION} REQUIRED)
    find_package(Protobuf REQUIRED)
    find_package(gz-plugin${GZ_PLUGIN_VERSION} REQUIRED COMPONENTS register)
    find_package(gz-sim${GZ_SIM_VERSION} REQUIRED)
    find_package(gz-sensors${GZ_SENSORS_VERSION} REQUIRED)

    # Create a flat output directory for all plugin libraries
    set(PX4_GZ_PLUGIN_OUTPUT_DIR "${CMAKE_CURRENT_BINARY_DIR}" CACHE PATH "Directory for all Gazebo plugin libraries")
    file(MAKE_DIRECTORY ${PX4_GZ_PLUGIN_OUTPUT_DIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PX4_GZ_PLUGIN_OUTPUT_DIR})

    # Add our plugins as subdirectories
    add_subdirectory(optical_flow)
    add_subdirectory(template_plugin)
    add_subdirectory(moving_platform_controller)

    # Add an alias target for each plugin
    add_custom_target(px4_gz_plugins ALL DEPENDS OpticalFlowSystem MovingPlatformController TemplatePlugin)
endif()
