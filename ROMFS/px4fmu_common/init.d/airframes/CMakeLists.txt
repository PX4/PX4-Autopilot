############################################################################
#
#   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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

include(px4_airframes)

px4_add_romfs_files(
	# [0-999] Reserved (historical)

	# [18000, 18999] High-altitude balloons
	18001_TF-B1

	# [22000, 22999] Reserve for custom models
)

if(CONFIG_MODULES_SIMULATION_PWM_OUT_SIM)
	px4_add_romfs_files(
		${SIMULATION_AIRFRAMES}
	)
endif()

if(CONFIG_MODULES_MC_RATE_CONTROL)
	px4_add_romfs_files(
		${MULTICOPTER_AIRFRAMES}
	)
endif()

if(CONFIG_MODULES_FW_RATE_CONTROL)
	px4_add_romfs_files(
		${FIXEDWING_AIRFRAMES}
	)
endif()

if(CONFIG_MODULES_AIRSHIP_ATT_CONTROL)
	px4_add_romfs_files(
		${AIRSHIP_AIRFRAMES}
	)
endif()

if(CONFIG_MODULES_VTOL_ATT_CONTROL)
	px4_add_romfs_files(
		${VTOL_AIRFRAMES}
	)
endif()

if(CONFIG_MODULES_ROVER_DIFFERENTIAL)
	px4_add_romfs_files(
		${DIFFERENTIAL_ROVER_AIRFRAMES}
	)
endif()

if(CONFIG_MODULES_ROVER_ACKERMANN)
	px4_add_romfs_files(
		${ACKERMANN_ROVER_AIRFRAMES}
	)
endif()

if(CONFIG_MODULES_ROVER_POS_CONTROL)
	px4_add_romfs_files(
		${ROVER_AIRFRAMES}
	)
endif()

if(CONFIG_MODULES_UUV_ATT_CONTROL)
	px4_add_romfs_files(
		${UUV_AIRFRAMES}
	)
endif()
