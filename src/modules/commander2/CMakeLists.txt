############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

include_directories(.)

set(srcs
	fsm_commander/StateManual.c
	fsm_commander/StateAuto.c
	fsm_commander/StateGuided.c
	fsm_commander/Commander.c
	)

if (NOT ${BOARD} STREQUAL "cbmc")
	list(APPEND srcs
		cbmc/cbmc_stub.c
		commander2_main.c
		)
endif()

px4_add_module(
	MODULE modules__commander2
	MAIN commander2
	STACK 5000
	SRCS ${srcs}
	DEPENDS
		platforms__common
		msg_gen
	)

add_executable(test_commander2 test_commander.c)
target_link_libraries(test_commander2 modules__commander2)

if (${BOARD} STREQUAL "cbmc")
	add_test(NAME test_commander2_assertion_coverage
		COMMAND ${cmake_tester}
		"${CBMC} test_commander2 --cover-assertions"
		--verbose
		--re-fail ".*failed to cover.*"
		)
	add_test(test_commander2_prop_verification ${CBMC} 
		--bounds-check
		--div-by-zero-check
		--pointer-check
		--memory-leak-check
		--signed-overflow-check
		--unsigned-overflow-check
		--nan-check
		test_commander2
		)
else()
	add_test(test_commander2 ${test_flags} test_commander2)
endif()
