############################################################################
#
#   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

FetchContent_Declare(
    sbgECom
    GIT_REPOSITORY https://github.com/SBG-Systems/sbgECom.git
    GIT_TAG 5.2.590-stable
    GIT_SHALLOW TRUE
    PATCH_COMMAND git apply --reverse --check "${CMAKE_CURRENT_LIST_DIR}/sbgecom.patch" || git apply --ignore-whitespace "${CMAKE_CURRENT_LIST_DIR}/sbgecom.patch"
)

FetchContent_MakeAvailable(sbgECom)

px4_add_module(
	MODULE drivers__ins__sbgecom
	MAIN sbgecom
	INCLUDES
		${sbgECom_SOURCE_DIR}/common
		${sbgECom_SOURCE_DIR}/src
	COMPILE_FLAGS
	SRCS
		sbgecom.cpp
		sbgecom.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		sbgECom
	)
