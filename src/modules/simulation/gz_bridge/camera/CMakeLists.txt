############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

find_package(gz-transport NAMES gz-transport13)
find_package(OpenCV REQUIRED)

include(ExternalProject)

ExternalProject_Add(OpticalFlow
    GIT_REPOSITORY https://github.com/PX4/PX4-OpticalFlow.git
    GIT_TAG master
    PREFIX ${CMAKE_BINARY_DIR}/OpticalFlow
    INSTALL_DIR ${CMAKE_BINARY_DIR}/OpticalFlow/install
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
    BUILD_BYPRODUCTS ${CMAKE_BINARY_DIR}/OpticalFlow/install/lib/libOpticalFlow.so
)

ExternalProject_Get_Property(OpticalFlow install_dir)

message(WARNING "OpticalFlow install dir: ${install_dir}")

set(OpticalFlow_INCLUDE_DIRS ${install_dir}/include)

# If Harmonic not found, look for GZ Garden libraries
if(NOT gz-transport_FOUND)
	find_package(gz-transport NAMES gz-transport12)
endif()

if(gz-transport_FOUND)

	add_compile_options(-frtti -fexceptions)

	set(GZ_TRANSPORT_VER ${gz-transport_VERSION_MAJOR})

	if(GZ_TRANSPORT_VER GREATER_EQUAL 12)
		set(GZ_TRANSPORT_LIB gz-transport${GZ_TRANSPORT_VER}::core)
	else()
		set(GZ_TRANSPORT_LIB ignition-transport${GZ_TRANSPORT_VER}::core)
	endif()

	px4_add_library(gz_camera
		gz_camera.hpp
		gz_camera.cpp
	)

	set(OpticalFlow_LIBS ${install_dir}/lib/libOpticalFlow.so)

	target_compile_options(gz_camera PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
	target_include_directories(gz_camera PRIVATE ${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIRS} ${OpticalFlow_INCLUDE_DIRS})
	target_link_libraries(gz_camera PRIVATE ${GZ_TRANSPORT_LIB} ${OpenCV_LIBS} ${OpticalFlow_LIBS})

	add_dependencies(gz_camera OpticalFlow)
endif()
