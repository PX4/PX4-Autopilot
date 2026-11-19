############################################################################
#
#   Copyright (c) 2018-2020 PX4 Development Team. All rights reserved.
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

px4_add_library(tecs
	TECS.cpp
	TECS.hpp
)

target_link_libraries(tecs PRIVATE geo)

# TECS takes its inputs as aggregates filled with designated initializers: flag any field left out
target_compile_options(tecs PRIVATE -Wmissing-field-initializers)

px4_add_functional_gtest(SRC TECSTest.cpp LINKLIBS tecs motion_planning)

px4_add_functional_gtest(SRC TECSClosedLoopTest.cpp LINKLIBS tecs motion_planning)

# Plot the closed-loop test cases: make px4_sitl_test tecs_closed_loop_plots
if(TARGET functional-TECSClosedLoop)
	set(TECS_PLOT_DIR ${PX4_BINARY_DIR}/tecs_closed_loop)
	add_custom_target(tecs_closed_loop_plots
		COMMAND ${CMAKE_COMMAND} -E remove_directory ${TECS_PLOT_DIR}
		COMMAND ${CMAKE_COMMAND} -E make_directory ${TECS_PLOT_DIR}/csv
		# Plot even if test cases fail, that is when the plots are needed most
		COMMAND sh -c "TECS_TEST_LOG_DIR=${TECS_PLOT_DIR}/csv $<TARGET_FILE:functional-TECSClosedLoop> || true"
		COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/plot_closed_loop.py ${TECS_PLOT_DIR}/csv -o ${TECS_PLOT_DIR}/png
		DEPENDS functional-TECSClosedLoop
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		COMMENT "Plotting the TECS closed-loop test cases into ${TECS_PLOT_DIR}/png"
		USES_TERMINAL
		VERBATIM
	)
endif()
