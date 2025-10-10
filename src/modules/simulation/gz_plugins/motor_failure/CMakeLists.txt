############################################################################
#
#   Copyright (c) 2017 Nuno Marques, PX4 Pro Dev Team, Lisbon
#   Copyright (c) 2017 Siddharth Patel, NTU Singapore
#   Copyright (c) 2022 SungTae Moon, KOREATECH Korea
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

project(MotorFailurePlugin)

# Save current BUILD_TESTING value
if(DEFINED BUILD_TESTING)
    set(_build_testing_backup ${BUILD_TESTING})
endif()

# Disable ament testing to avoid conflicts with PX4's test system
set(BUILD_TESTING OFF CACHE BOOL "Disable ament testing" FORCE)

# Find ROS2 packages
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Restore BUILD_TESTING value
if(DEFINED _build_testing_backup)
    set(BUILD_TESTING ${_build_testing_backup} CACHE BOOL "Enable testing" FORCE)
    unset(_build_testing_backup)
else()
    unset(BUILD_TESTING CACHE)
endif()

add_library(${PROJECT_NAME} SHARED
    MotorFailureSystem.cpp
)

# Include ROS2 directories as SYSTEM to suppress warnings
target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC px4_gz_msgs
    SYSTEM PUBLIC ${rclcpp_INCLUDE_DIRS}
    SYSTEM PUBLIC ${std_msgs_INCLUDE_DIRS}
)

# Disable specific warnings for ROS2 headers compatibility
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wno-shadow
    -Wno-error=shadow
)

# Link libraries - follow Gazebo Classic approach
target_link_libraries(${PROJECT_NAME}
    PUBLIC px4_gz_msgs
    PUBLIC ${GZ_SENSORS_TARGET}
    PUBLIC ${GZ_PLUGIN_TARGET}
    PUBLIC ${GZ_SIM_TARGET}
    PUBLIC ${GZ_TRANSPORT_TARGET}
    PUBLIC ${rclcpp_LIBRARIES}
    PUBLIC ${std_msgs_LIBRARIES}
)
