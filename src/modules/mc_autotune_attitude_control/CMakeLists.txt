############################################################################
#
#   Copyright (c) 2020-2021 PX4 Development Team. All rights reserved.
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

# Select one implementation so constrained boards retain standard Autotune.
set(autotune_path .)
set(autotune_config mc_autotune_attitude_control_params.yaml)
set(autotune_compile_flags ${MAX_CUSTOM_OPT_LEVEL})
if(CONFIG_MC_AUTOTUNE_EXPERIMENTAL)
	set(autotune_path experimental)
	list(APPEND autotune_config experimental/mc_autotune_attitude_control_params.yaml)
	set(autotune_compile_flags)
endif()

px4_add_module(
	MODULE mc_autotune_attitude_control
	MAIN mc_autotune_attitude_control
	COMPILE_FLAGS ${autotune_compile_flags}
	SRCS
		${autotune_path}/mc_autotune_attitude_control.cpp
		${autotune_path}/mc_autotune_attitude_control.hpp
	MODULE_CONFIG
		${autotune_config}
	DEPENDS
		hysteresis
		mathlib
		px4_work_queue
		SystemIdentification
)

set(standard_test_library mc_autotune_attitude_control)
if(CONFIG_MC_AUTOTUNE_EXPERIMENTAL)
	px4_add_functional_gtest(
		SRC experimental/McAutotuneResponseTest.cpp
		LINKLIBS mc_autotune_attitude_control
	)
	if(BUILD_TESTING)
		# Exercise the standard lifecycle in a separate executable as well.
		px4_add_library(mc_autotune_standard_test mc_autotune_attitude_control.cpp)
		target_link_libraries(mc_autotune_standard_test PRIVATE hysteresis mathlib px4_work_queue SystemIdentification)
		set(standard_test_library mc_autotune_standard_test)
	endif()
endif()

px4_add_functional_gtest(
	SRC McAutotuneAttitudeControlTest.cpp
	LINKLIBS ${standard_test_library}
)
px4_add_unit_gtest(SRC ControllerValidationTest.cpp LINKLIBS mathlib)
px4_add_functional_gtest(
	SRC AutotuneVtolTest.cpp
	LINKLIBS mc_autotune_attitude_control fw_autotune_attitude_control
)
