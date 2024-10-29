############################################################################
#
#   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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


set(MIP_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mip_sdk)
px4_add_git_submodule(TARGET git_mip PATH ${MIP_SDK_DIR})

# Set some options for the MIP SDK which we will reset after building.
set(MIP_USE_TCP "OFF" CACHE BOOL "" FORCE)
set(MIP_DISABLE_CPP "ON" CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES "OFF" CACHE BOOL "" FORCE)
set(BUILD_TESTING "OFF" CACHE BOOL "" FORCE)
add_subdirectory(mip_sdk)

project(drivers__ins__microstrain LANGUAGES C CXX)

px4_add_module(
	MODULE drivers__ins__microstrain
	MAIN microstrain
	COMPILE_FLAGS
	SRCS
		MicroStrain.cpp
		MicroStrain.hpp
		modal_io_serial.cpp
		modal_io_serial.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		px4_work_queue
		mip
	)
