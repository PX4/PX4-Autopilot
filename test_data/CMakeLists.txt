############################################################################
#
#   Copyright (c) 2016 PX4 Development Team. All rights reserved.
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

set(test_data_file_names
	dsm_x_data.txt
	dsm_x_dx9_data.txt
	sbus2_r7008SB.txt
	st24_data.txt
	sumd_data.txt
)

# Get absolute paths
set(test_data_files)
foreach(test_data_file ${test_data_file_names})
	list(APPEND test_data_files ${CMAKE_CURRENT_SOURCE_DIR}/${test_data_file})
endforeach()

set(test_data_directory ${ROOTFS_DIRECTORY}/test_data/)

add_custom_target(test_data_dir
	COMMAND ${CMAKE_COMMAND} -E make_directory ${test_data_directory}
)

# cmake -E copy with version < 3.5 does not support copying multiple files yet, therefore use cp.
add_custom_target(test_data
	COMMAND cp ${test_data_files} ${test_data_directory}
	DEPENDS test_data_dir
)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
