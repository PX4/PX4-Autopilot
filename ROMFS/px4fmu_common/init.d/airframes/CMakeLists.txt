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

px4_add_romfs_files(
	# [0-999] Reserved (historical)

	# [18000, 18999] High-altitude balloons
	18001_TF-B1

	# [22000, 22999] Reserve for custom models
)

if(CONFIG_MODULES_SIMULATION_PWM_OUT_SIM)
	px4_add_romfs_files(
		# [1000, 1999] Simulation setups
		1001_rc_quad_x.hil
		1002_standard_vtol.hil
		1100_rc_quad_x_sih.hil
		1101_rc_plane_sih.hil
		1102_tailsitter_duo_sih.hil
		1103_standard_vtol_sih.hil
	)
endif()

if(CONFIG_MODULES_MC_RATE_CONTROL)
	px4_add_romfs_files(
		# [4000, 4999] Quadrotor x
		4001_quad_x
		4014_s500
		4015_holybro_s500
		4016_holybro_px4vision
		4017_nxp_hovergames
		4019_x500_v2
		4020_holybro_px4vision_v1_5
		4041_beta75x
		4050_generic_250
		4052_holybro_qav250
		4053_holybro_kopis2
		4061_atl_mantis_edu
		4071_ifo
		4073_ifo-s
		4500_clover4
		4601_droneblocks_dexi_5
		4901_crazyflie21

		# [5000, 5999] Quadrotor +
		5001_quad_+

		# [6000, 6999] Hexarotor x
		6001_hexa_x
		6002_draco_r

		# [7000, 7999] Hexarotor +
		7001_hexa_+

		# [8000, 8999] Octorotor +
		8001_octo_x

		# [9000, 9999] Octorotor +
		9001_octo_+

		# [11000, 11999] Hexa Cox
		11001_hexa_cox

		# [12000, 12999] Octo Cox
		12001_octo_cox

		# [14000, 14999] MC with tilt
		14001_generic_mc_with_tilt

		16001_helicopter

		24001_dodeca_cox
	)
endif()

if(CONFIG_MODULES_FW_RATE_CONTROL)
	px4_add_romfs_files(
		# [2000, 2999] Standard planes
		2100_standard_plane
		2106_albatross

		# [3000, 3999] Flying wing
		3000_generic_wing

		# [17000, 17999] Autogyro
		17002_TF-AutoG2
		17003_TF-G2
	)
endif()

if(CONFIG_MODULES_AIRSHIP_ATT_CONTROL)
	px4_add_romfs_files(
		2507_cloudship
	)
endif()

if(CONFIG_MODULES_VTOL_ATT_CONTROL)
	px4_add_romfs_files(
		# [13000, 13999] VTOL
		13000_generic_vtol_standard
		13100_generic_vtol_tiltrotor
		13030_generic_vtol_quad_tiltrotor
		13200_generic_vtol_tailsitter
	)
endif()

if(CONFIG_MODULES_ROVER_DIFFERENTIAL)
	px4_add_romfs_files(
		# [50000, 50999] Differential rovers
		50000_generic_rover_differential
		50001_aion_robotics_r1_rover
	)
endif()

if(CONFIG_MODULES_ROVER_ACKERMANN)
	px4_add_romfs_files(
		# [51000, 51999] Ackermann rovers
		51000_generic_rover_ackermann
		51001_axial_scx10_2_trail_honcho
	)
endif()

if(CONFIG_MODULES_ROVER_MECANUM)
	px4_add_romfs_files(
		# [52000, 52999] Mecanum rovers
		52000_generic_rover_mecanum
	)
endif()

if(CONFIG_MODULES_ROVER_POS_CONTROL)
	px4_add_romfs_files(
		# [59000, 59999] Rover position control (deprecated)
		59000_generic_ground_vehicle
		59001_nxpcup_car_dfrobot_gpx
	)
endif()

if(CONFIG_MODULES_UUV_ATT_CONTROL)
	px4_add_romfs_files(
		# [60000, 61000] (Unmanned) Underwater Robots
		60000_uuv_generic
		60001_uuv_hippocampus
		60002_uuv_bluerov2_heavy
	)
endif()
