############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

px4_add_library(MotorFailureDetector
	MotorFailureDetector.cpp
	MotorFailureDetector.hpp
)
target_include_directories(MotorFailureDetector PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

px4_add_unit_gtest(SRC MotorFailureDetectorTest.cpp LINKLIBS MotorFailureDetector mathlib hysteresis)

# Offline log-replay tool: drives the real detector over a recorded ulog (host only). Opt-in (off by
# default) via -DMFD_REPLAY_TOOL=ON so a normal build never fetches/builds it or its ulog_cpp submodule.
set(MFD_REPLAY_TOOL OFF CACHE BOOL "Build the host-only MotorFailureDetector offline log-replay tool")

if(${PX4_PLATFORM} MATCHES "posix" AND MFD_REPLAY_TOOL)
	px4_add_git_submodule(TARGET git_ulog_cpp PATH "${PX4_SOURCE_DIR}/src/lib/ulog_cpp")
	add_subdirectory(${PX4_SOURCE_DIR}/src/lib/ulog_cpp ${CMAKE_CURRENT_BINARY_DIR}/ulog_cpp EXCLUDE_FROM_ALL)

	# ulog_cpp is third-party: silence the specific warnings it trips under PX4's -Werror set
	# (-Wshadow in its code, -Warray-bounds a libstdc++ false positive) and mark its headers system.
	target_compile_options(ulog_cpp PRIVATE -Wno-shadow -Wno-array-bounds)
	get_target_property(_ulogcpp_inc ulog_cpp INTERFACE_INCLUDE_DIRECTORIES)
	set_target_properties(ulog_cpp PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_ulogcpp_inc}")

	add_executable(mfd_replay EXCLUDE_FROM_ALL tools/mfd_replay.cpp)
	target_link_libraries(mfd_replay PRIVATE MotorFailureDetector mathlib hysteresis ulog_cpp::ulog_cpp)
endif()
