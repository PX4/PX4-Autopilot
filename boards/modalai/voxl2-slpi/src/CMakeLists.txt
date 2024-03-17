############################################################################
#
#   Copyright (c) 2022 ModalAI, Inc. All rights reserved.
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

# Need to make sure that the DSP processor on VOXL2
# knows about all parameters since some modules need parameters
# from other modules that are not running on the DSP.
set(DISABLE_PARAMS_MODULE_SCOPING TRUE PARENT_SCOPE)

add_library(drivers_board
	board_config.h
	i2c.cpp
	init.c
	spi.cpp
    )

# Add custom drivers for SLPI
add_subdirectory(${PX4_BOARD_DIR}/src/drivers/rc_controller)
add_subdirectory(${PX4_BOARD_DIR}/src/drivers/mavlink_rc_in)
# add_subdirectory(${PX4_BOARD_DIR}/src/drivers/spektrum_rc)
# add_subdirectory(${PX4_BOARD_DIR}/src/drivers/ghst_rc)
add_subdirectory(${PX4_BOARD_DIR}/src/drivers/dsp_hitl)
# add_subdirectory(${PX4_BOARD_DIR}/src/drivers/dsp_sbus)
add_subdirectory(${PX4_BOARD_DIR}/src/drivers/elrs_led)
