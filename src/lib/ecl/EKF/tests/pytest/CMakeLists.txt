############################################################################
#
#   Copyright (c) 2015-2018 ECL Development Team. All rights reserved.
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

if(EKF_PYTHON_TESTS)

	# pip3 install -r requirements.txt 

	add_custom_target(ecl_EKF_pytest
		COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_BINARY_DIR}/EKF/swig/python ${CMAKE_CURRENT_SOURCE_DIR}/ekf_test.py --verbose --plots
		DEPENDS ecl_EKF _ecl_EKF
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
		USES_TERMINAL
	)

	add_custom_target(ecl_EKF_pytest-quick
		COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_BINARY_DIR}/EKF/swig/python ${CMAKE_CURRENT_SOURCE_DIR}/ekf_test.py --quick --verbose --plots
		DEPENDS ecl_EKF _ecl_EKF
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}		
		USES_TERMINAL
	)

	add_custom_target(ecl_EKF_pytest-benchmark
		COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_BINARY_DIR}/EKF/swig/python ${CMAKE_CURRENT_SOURCE_DIR}/ekf_test.py --benchmark
		DEPENDS ecl_EKF _ecl_EKF
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}		
		USES_TERMINAL
	)

	# requires python3-tk
	add_custom_target(ecl_EKF_pytest-plots
		COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_BINARY_DIR}/EKF/swig/python ${CMAKE_CURRENT_SOURCE_DIR}/ekf_test.py --plots
		DEPENDS ecl_EKF _ecl_EKF
		WORKING_DIRECTORY ${CMAKE_BINARY_DIR}		
		USES_TERMINAL
	)

	add_custom_target(ecl_EKF_pytest-lint
		COMMAND ${CMAKE_COMMAND} -E env PYTHONPATH=${CMAKE_BINARY_DIR}/EKF/swig/python ${CMAKE_CURRENT_SOURCE_DIR}/lint.py
		DEPENDS ecl_EKF _ecl_EKF
		WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
		USES_TERMINAL
	)
	
	# ctest entry
	add_test(NAME ecl_EKF_pytest_run
		COMMAND ${CMAKE_COMMAND} --build ${CMAKE_BINARY_DIR} --target ecl_EKF_pytest-quick
		)

endif()
