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

project(OpticalFlowSystem)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

set(PX4_OPTICAL_FLOW_DIR ${CMAKE_CURRENT_SOURCE_DIR}/PX4-OpticalFlow)

px4_add_git_submodule(TARGET git_px4_optical_flow PATH "${PX4_OPTICAL_FLOW_DIR}")

if(NOT TARGET OpticalFlow)
    add_subdirectory(${PX4_OPTICAL_FLOW_DIR} ${CMAKE_CURRENT_BINARY_DIR}/PX4-OpticalFlow EXCLUDE_FROM_ALL)
    add_dependencies(OpticalFlow git_px4_optical_flow)
    add_dependencies(klt_feature_tracker git_px4_optical_flow)

    # PX4-OpticalFlow is an external dependency that predates PX4's strict
    # warning policy. Relax -Werror and the few specific warnings it trips so
    # upstream source builds cleanly inside our tree. Also restore default
    # symbol visibility so the shared library exports its C++ class symbols
    # (PX4's global CXX flags set -fvisibility=hidden).
    foreach(_tgt OpticalFlow klt_feature_tracker)
        target_compile_options(${_tgt} PRIVATE
            -Wno-error
            -Wno-sign-compare
            -Wno-cast-align
            -Wno-shadow
            -fvisibility=default
        )
        set_target_properties(${_tgt} PROPERTIES
            CXX_VISIBILITY_PRESET default
            VISIBILITY_INLINES_HIDDEN OFF
        )
    endforeach()
endif()

add_library(${PROJECT_NAME} SHARED
    OpticalFlowSensor.cpp
    OpticalFlowSystem.cpp
)

target_link_libraries(${PROJECT_NAME}
    PUBLIC px4_gz_msgs
    PUBLIC ${GZ_SENSORS_TARGET}
    PUBLIC ${GZ_PLUGIN_TARGET}
    PUBLIC ${GZ_SIM_TARGET}
    PUBLIC ${GZ_TRANSPORT_TARGET}
    PUBLIC ${OpenCV_LIBS}
    PUBLIC OpticalFlow
)

target_include_directories(${PROJECT_NAME}
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
    PUBLIC ${CMAKE_CURRENT_BINARY_DIR}
    PUBLIC ${OpenCV_INCLUDE_DIRS}
    PUBLIC ${PX4_OPTICAL_FLOW_DIR}/include
    PUBLIC ${PX4_OPTICAL_FLOW_DIR}/external/klt_feature_tracker/include
    PUBLIC px4_gz_msgs
)

# libOpticalFlow.so is built alongside the plugin in the same output directory
# (CMAKE_LIBRARY_OUTPUT_DIRECTORY is set by the parent gz_plugins CMakeLists),
# so $ORIGIN is sufficient at both build and install time.
set_target_properties(${PROJECT_NAME} PROPERTIES
    BUILD_RPATH "$ORIGIN"
    INSTALL_RPATH "$ORIGIN"
)

if (NOT CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    install(TARGETS ${PROJECT_NAME} OpticalFlow LIBRARY DESTINATION lib/px4_gz_plugins)
endif()
