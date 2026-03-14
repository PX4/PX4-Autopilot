############################################################################
#
#   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

# VOXL2 board-specific install rules for .deb packaging
# Included from platforms/posix/CMakeLists.txt where the px4 target exists

# SLPI companion build output directory
set(VOXL2_SLPI_BUILD_DIR "${PX4_SOURCE_DIR}/build/modalai_voxl2-slpi_default")

# Apps processor binary
install(TARGETS px4 RUNTIME DESTINATION bin)

# px4-alias.sh (generated during build into bin/ subdirectory)
install(PROGRAMS ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/px4-alias.sh DESTINATION bin)

# Startup scripts from board target directory
install(PROGRAMS
	${PX4_BOARD_DIR}/target/voxl-px4
	${PX4_BOARD_DIR}/target/voxl-px4-start
	${PX4_BOARD_DIR}/target/voxl-px4-hitl
	${PX4_BOARD_DIR}/target/voxl-px4-hitl-start
	DESTINATION bin
)

# DSP firmware blob from companion SLPI build
install(FILES ${VOXL2_SLPI_BUILD_DIR}/platforms/qurt/libpx4.so
	DESTINATION lib/rfsa/adsp
	OPTIONAL
)

# Configuration files
install(FILES
	${PX4_BOARD_DIR}/target/voxl-px4-fake-imu-calibration.config
	${PX4_BOARD_DIR}/target/voxl-px4-hitl-set-default-parameters.config
	DESTINATION ../etc/modalai
)

# Systemd service file
install(FILES ${PX4_BOARD_DIR}/debian/voxl-px4.service
	DESTINATION ../etc/systemd/system
)

# Component metadata JSON files
install(FILES
	${PX4_BINARY_DIR}/actuators.json.xz
	${PX4_BINARY_DIR}/component_general.json.xz
	${PX4_BINARY_DIR}/parameters.json.xz
	DESTINATION ../data/px4/etc/extras
	OPTIONAL
)
install(FILES ${PX4_BINARY_DIR}/events/all_events.json.xz
	DESTINATION ../data/px4/etc/extras
	OPTIONAL
)
