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

project(GstCameraSystem)

# Find required packages
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER gstreamer-1.0)
pkg_check_modules(GSTREAMER_APP gstreamer-app-1.0)

if (NOT GSTREAMER_FOUND OR NOT GSTREAMER_APP_FOUND)
    message(WARNING "GStreamer and Gstreamer App are required to build this plugin.")
else()
    add_library(${PROJECT_NAME} SHARED
        GstCameraSystem.cpp
    )

    target_link_libraries(${PROJECT_NAME}
        PUBLIC px4_gz_msgs
        PUBLIC gz-sensors${gz-sensors_VERSION_MAJOR}::gz-sensors${gz-sensors_VERSION_MAJOR}
        PUBLIC gz-plugin${gz-plugin_VERSION_MAJOR}::gz-plugin${gz-plugin_VERSION_MAJOR}
        PUBLIC gz-sim${gz-sim_VERSION_MAJOR}::gz-sim${gz-sim_VERSION_MAJOR}
        PUBLIC gz-transport${gz-transport_VERSION_MAJOR}::gz-transport${gz-transport_VERSION_MAJOR}
        PUBLIC ${GSTREAMER_LIBRARIES}
        PUBLIC ${GSTREAMER_APP_LIBRARIES}
    )

    target_include_directories(${PROJECT_NAME}
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
        PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
        PUBLIC px4_gz_msgs
        PUBLIC ${GSTREAMER_INCLUDE_DIRS}
        PUBLIC ${GSTREAMER_APP_INCLUDE_DIRS}
    )

    target_compile_options(${PROJECT_NAME}
        PRIVATE ${GSTREAMER_CFLAGS}
        PRIVATE ${GSTREAMER_APP_CFLAGS}
    )
endif()
