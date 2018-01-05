############################################################################
#
#   Copyright (c) 2015 ECL Development Team. All rights reserved.
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
# 3. Neither the name ECL nor the names of its contributors may be
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

cmake_minimum_required(VERSION 2.8)

project(ECL CXX)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR})

set(MATRIX_DIR ${CMAKE_SOURCE_DIR}/../matrix)
if(NOT EXISTS "${MATRIX_DIR}/.git")
	message(SEND_ERROR "The git submodules are not available. Please run git submodule update --init --recursive")
endif()

include_directories(
	..
	${MATRIX_DIR}
	)

set(SRCS
	airspeed_fusion.cpp
	control.cpp
	covariance.cpp
	drag_fusion.cpp
	ekf.cpp
	ekf_helper.cpp
	estimator_interface.cpp
	geo.cpp
	gps_checks.cpp
	mag_fusion.cpp
	mathlib.cpp
	optflow_fusion.cpp
	sideslip_fusion.cpp
	terrain_estimator.cpp
	vel_pos_fusion.cpp
	)

add_definitions(-DPOSIX_SHARED)
add_compile_options(
	-pedantic
	-std=c++11

	-Wall
	-Werror

	-Wno-deprecated-declarations
	-Wno-enum-compare
	-Wno-unused-local-typedefs
	-Wno-unused-parameter
	)

# clang tolerate unknown gcc options
if("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
	add_compile_options(-Wno-unknown-warning-option)
endif()

add_library(ecl SHARED ${SRCS})

# Python bindings & tests
# Use cmake -DPythonTests=1 ../EKF && make pytest
if(PythonTests)
  # Use Python 3.5
  set(Python_ADDITIONAL_VERSIONS 3.5)
  find_package(PythonLibs 3 REQUIRED)
  find_package(PythonInterp 3 REQUIRED)

  # Need SWIG to wrap ecl
  find_package(SWIG REQUIRED)
  include(${SWIG_USE_FILE})

  add_definitions(-DSWIG_TYPE_TABLE=ecl)
  set_source_files_properties(../swig/ecl.i PROPERTIES CPLUSPLUS ON)
  set(swigged_sources ekf.h)

  # Find numpy include
  execute_process(COMMAND ${PYTHON_EXECUTABLE} -c "import numpy; print(numpy.get_include())" OUTPUT_VARIABLE NUMPY_INCLUDE_DIRS ERROR_QUIET)

  include_directories(
    ..
    ${MATRIX_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${PYTHON_INCLUDE_DIRS}
    ${NUMPY_INCLUDE_DIRS}
    )

  set(SWIG_MODULE_ecl_EXTRA_DEPS ${swigged_sources})
  swig_add_module(ecl python ../swig/ecl.i)
  swig_link_libraries(ecl ${PYTHON_LIBRARIES} ecl)

  add_custom_target(pytest
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_SOURCE_DIR}/tests/pytest/ekf_test --verbose
    )
  add_dependencies(pytest ${SWIG_MODULE_ecl_REAL_NAME})

  add_custom_target(pytest-quick
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_SOURCE_DIR}/tests/pytest/ekf_test --quick --verbose
    )
  add_dependencies(pytest-quick ${SWIG_MODULE_ecl_REAL_NAME})

  add_custom_target(pytest-benchmark
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_SOURCE_DIR}/tests/pytest/ekf_test --benchmark
    )
  add_dependencies(pytest-benchmark ${SWIG_MODULE_ecl_REAL_NAME})

  add_custom_target(pytest-plots
    env PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR} ${CMAKE_SOURCE_DIR}/tests/pytest/ekf_test --plots
    )
  add_dependencies(pytest-plots ${SWIG_MODULE_ecl_REAL_NAME})

  add_custom_target(pytest-lint
    COMMAND cd  ${CMAKE_SOURCE_DIR}/tests/pytest/ && ${CMAKE_SOURCE_DIR}/tests/pytest/lint
    )
  add_dependencies(pytest-lint ${SWIG_MODULE_ecl_REAL_NAME})
endif()
