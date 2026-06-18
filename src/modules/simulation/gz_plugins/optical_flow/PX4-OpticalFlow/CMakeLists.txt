############################################################################
#
#   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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

cmake_minimum_required(VERSION 3.6)

project(OpticalFlow)
set(version_major 0)
set(version_minor 1)
set(version_patch 0)
set(version "${version_major}.${version_minor}.${version_patch}")
set(package-contact "christoph@px4.io")

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpic")

set(${PROJECT_NAME}_INCLUDE_DIRS
    ${PROJECT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

find_package( OpenCV REQUIRED )
OPTION(OPENCV "ON for OpenCV" ON) # -DOPENCV=OFF

add_subdirectory( external/klt_feature_tracker klt_feature_tracker )
set( klt_feature_tracker_LIBS "klt_feature_tracker" )

if(OpenCV_FOUND AND OPENCV)
    message(STATUS "Building ${PROJECT_NAME} with OpenCV")

    set(OpticalFlow_INCLUDE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/include
      ${klt_feature_tracker_INCLUDE_DIRS}
      ${OpenCV_INCLUDE_DIRS}
    )

    include_directories(OpticalFlow_INCLUDE_DIRS)

    add_library( OpticalFlow SHARED
      src/optical_flow.cpp
      src/px4flow.cpp
     	src/flow_px4.cpp
    	src/flow_opencv.cpp
    )

    target_link_libraries( OpticalFlow
      ${OpenCV_LIBS}
      ${klt_feature_tracker_LIBS}
    )
else()
    message(STATUS "Building ${PROJECT_NAME} without OpenCV")

    set(OpticalFlow_INCLUDE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}/include
    )

    include_directories(OpticalFlow_INCLUDE_DIRS)

    add_library( OpticalFlow SHARED
      src/optical_flow.cpp
      src/px4flow.cpp
     	src/flow_px4.cpp
    )

endif()

target_include_directories(OpticalFlow PUBLIC include)

# Generate the Config file from the .in file, doing @VAR@ substitution only
configure_file(OpticalFlowConfig.cmake.in FindOpticalFlow.cmake @ONLY)

# INSTALL STEPS
INSTALL(
  TARGETS OpticalFlow
  RUNTIME DESTINATION lib
  LIBRARY DESTINATION lib)

INSTALL(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/external/klt_feature_tracker/include/
  DESTINATION include)

INSTALL(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/
  DESTINATION include)

INSTALL(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/FindOpticalFlow.cmake
  DESTINATION ${CMAKE_INSTALL_PREFIX})
