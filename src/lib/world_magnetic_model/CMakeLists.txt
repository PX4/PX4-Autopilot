############################################################################
#
#   Copyright (c) 2018-2022 PX4 Development Team. All rights reserved.
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

add_custom_target(world_magnetic_model_update
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/fetch_noaa_table.py > ${CMAKE_CURRENT_SOURCE_DIR}/geo_magnetic_tables.hpp
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/fetch_noaa_table.py
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	USES_TERMINAL
	COMMENT "Updating world magnetic model from NOAA (${CMAKE_CURRENT_SOURCE_DIR}/geo_magnetic_tables.hpp)"
)

add_custom_target(world_magnetic_model_tests_update
	COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/generate_gtest.py > ${CMAKE_CURRENT_SOURCE_DIR}/test_geo_lookup.cpp
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/generate_gtest.py
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	USES_TERMINAL
	COMMENT "Updating world magnetic model unit tests from NOAA (${CMAKE_CURRENT_SOURCE_DIR}/test_geo_lookup.cpp)"
)

add_library(world_magnetic_model
	geo_mag_declination.cpp
	geo_mag_declination.h
	geo_magnetic_tables.hpp
)
add_dependencies(world_magnetic_model prebuild_targets)
target_compile_options(world_magnetic_model PRIVATE ${MAX_CUSTOM_OPT_LEVEL})

if(BUILD_TESTING)
	px4_add_unit_gtest(SRC test_geo_lookup.cpp LINKLIBS world_magnetic_model)
	target_compile_options(unit-test_geo_lookup PRIVATE -O0 -Wno-double-promotion)
endif()
