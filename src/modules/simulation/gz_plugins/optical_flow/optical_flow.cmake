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

include(ExternalProject)
find_package(OpenCV REQUIRED)

if(NOT TARGET OpticalFlow)
    ExternalProject_Add(OpticalFlow
        GIT_REPOSITORY https://github.com/PX4/PX4-OpticalFlow.git
        GIT_TAG master
        PREFIX ${CMAKE_BINARY_DIR}/OpticalFlow
        INSTALL_DIR ${CMAKE_BINARY_DIR}/OpticalFlow/install
        CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=<INSTALL_DIR>
        BUILD_BYPRODUCTS ${CMAKE_BINARY_DIR}/OpticalFlow/install/lib/libOpticalFlow.so
        UPDATE_DISCONNECTED ON
        BUILD_ALWAYS OFF
        STEP_TARGETS build
    )

    ExternalProject_Get_Property(OpticalFlow install_dir)
    set(OpticalFlow_INCLUDE_DIRS ${install_dir}/include CACHE INTERNAL "")
    set(OpticalFlow_LIBS ${install_dir}/lib/libOpticalFlow.so CACHE INTERNAL "")
endif()
