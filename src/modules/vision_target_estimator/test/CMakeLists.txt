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

set(VTEST_TEST_COMMON_INCLUDES
	${CMAKE_CURRENT_SOURCE_DIR}/..
)

px4_add_unit_gtest(
	SRC TEST_VTEOosm.cpp
	INCLUDES ${VTEST_TEST_COMMON_INCLUDES}
)

px4_add_unit_gtest(
	SRC TEST_VTE_KF_position.cpp
	INCLUDES ${VTEST_TEST_COMMON_INCLUDES}
	LINKLIBS modules__vision_target_estimator
)

px4_add_unit_gtest(
	SRC TEST_VTE_KF_orientation.cpp
	INCLUDES ${VTEST_TEST_COMMON_INCLUDES}
	LINKLIBS modules__vision_target_estimator
)

px4_add_functional_gtest(
	SRC TEST_VTEPosition.cpp
	INCLUDES ${VTEST_TEST_COMMON_INCLUDES}
	LINKLIBS modules__vision_target_estimator
	         geo
)

px4_add_functional_gtest(
	SRC TEST_VTEOrientation.cpp
	INCLUDES ${VTEST_TEST_COMMON_INCLUDES}
	LINKLIBS modules__vision_target_estimator
)

px4_add_functional_gtest(
	SRC TEST_VTE_VisionTargetEst.cpp
	INCLUDES ${VTEST_TEST_COMMON_INCLUDES}
	LINKLIBS modules__vision_target_estimator
	         geo
)

foreach(vtest_target
		unit-TEST_VTEOosm
		unit-TEST_VTE_KF_position
		unit-TEST_VTE_KF_orientation
		functional-TEST_VTEPosition
		functional-TEST_VTEOrientation
		functional-TEST_VTE_VisionTargetEst
)
	add_dependencies(${vtest_target} uorb_headers)
	target_compile_definitions(${vtest_target} PRIVATE VTE_ENABLE_TEST_HOOKS=1)
endforeach()

# These tests include headers that pull in the generated position state directly.
foreach(vtest_target
		unit-TEST_VTE_KF_position
		functional-TEST_VTEPosition
		functional-TEST_VTE_VisionTargetEst
)
	add_dependencies(${vtest_target} ${VTEST_GENERATED_TARGET})
endforeach()
