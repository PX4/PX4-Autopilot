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

add_compile_definitions(SBG_CONFIG_LOG_MAX_SIZE=128)

set(SBGECOM_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sbgECom)
px4_add_git_submodule(TARGET git_sbgECom PATH ${SBGECOM_DIR})

add_subdirectory(sbgECom)

add_dependencies(sbgECom prebuild_targets)

target_compile_options(sbgECom
        PRIVATE
        -Wno-format
        -Wno-format-security
        -Wno-bad-function-cast
        -Wno-double-promotion
        -Wno-type-limits
        -Wno-maybe-uninitialized
        -Wno-float-equal
)

if("${PX4_PLATFORM}" MATCHES "nuttx")
    target_compile_definitions(sbgECom PUBLIC __NUTTX__)
endif()


px4_add_module(
	MODULE drivers__ins__sbgecom
	MAIN sbgecom
	INCLUDES
		sbgECom/common
		sbgECom/src
	COMPILE_FLAGS
	SRCS
		sbgecom.cpp
		sbgecom.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		sbgECom
	)
