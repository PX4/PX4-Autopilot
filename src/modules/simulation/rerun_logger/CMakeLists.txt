############################################################################
#
#   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

include(FetchContent)

# px4-rerun is pulled in via FetchContent (not a git submodule) so the SITL
# build is self-contained. Pin a released tag for reproducibility.
set(PX4_RERUN_LOADER OFF CACHE BOOL "" FORCE)
set(PX4_RERUN_TERRAIN OFF CACHE BOOL "" FORCE)

FetchContent_Declare(px4_rerun
	GIT_REPOSITORY https://github.com/breuerpeter/px4-rerun.git
	GIT_TAG v0.1.0)
FetchContent_MakeAvailable(px4_rerun)

# Suppress PX4's strict warnings for third-party Rerun SDK code
target_compile_options(rerun_sdk PRIVATE
	-Wno-double-promotion
	-Wno-float-equal
	-Wno-shadow
	-Wno-error
)

px4_add_module(
	MODULE modules__simulation__rerun_logger
	MAIN rerun_logger
	SRCS
		RerunLogger.cpp
		RerunLogger.hpp
	DEPENDS
		px4_work_queue
		geo
		px4_rerun
	)

target_compile_features(modules__simulation__rerun_logger PRIVATE cxx_std_17)
