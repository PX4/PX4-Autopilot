############################################################################
#
#   Copyright (c) 2015-2023 PX4 Development Team. All rights reserved.
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
#############################################################################

set(EKF_BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../ekf2)

set(EKF_DERIVATION_SRC_DIR ${EKF_BASE_DIR}/EKF/python/ekf_derivation)

set(EKF_GENERATED_FILES ${EKF_DERIVATION_SRC_DIR}/generated/state.h)
set(EKF_GENERATED_DERIVATION_INCLUDE_PATH "${EKF_DERIVATION_SRC_DIR}/..")

set(NAVPUTER_MODULE_PARAMS)
set(NAVPUTER_LIBS)
set(NAVPUTER_SRCS
	navputer.cpp
	navputer.hpp
)

px4_add_module(
	MODULE modules__navputer
	MAIN navputer
	COMPILE_FLAGS
		${MAX_CUSTOM_OPT_LEVEL}
		-fno-associative-math
		#-DDEBUG_BUILD
		#-O0
	INCLUDES
		${EKF_BASE_DIR}
		${EKF_BASE_DIR}/EKF
		${EKF_GENERATED_DERIVATION_INCLUDE_PATH}
	PRIORITY
		"SCHED_PRIORITY_MAX - 18" # max priority below high priority WQ threads
	STACK_MAX
		3600
	SRCS
		${NAVPUTER_SRCS}
		${EKF_GENERATED_FILES}

	MODULE_CONFIG
		module.yaml
		${NAVPUTER_MODULE_PARAMS}

	DEPENDS
		${NAVPUTER_LIBS}
	UNITY_BUILD
	)

