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
	# init scripts
	rc.fw_apps
	rc.fw_defaults
	rc.interface
	rc.io
	rc.logging
	rc.mc_apps
	rc.mc_defaults
	rcS
	rc.sensors
	rc.ugv_apps
	rc.ugv_defaults
	rc.vtol_apps
	rc.vtol_defaults

	# HITL airframes
	1000_rc_fw_easystar.hil
	1001_rc_quad_x.hil
	1002_standard_vtol.hil
	
	# Standard planes
	2100_standard_plane
	2105_maja
	2106_albatross
	
	# Wings
	3000_generic_wing
	3030_io_camflyer
	3031_phantom
	3032_skywalker_x5
	3033_wingwing
	3034_fx79
	3035_viper
	3036_pigeon
	3037_parrot_disco_mod
	3100_tbs_caipirinha
	
	# Quadrotors
	4001_quad_x
	4014_s500
	4030_3dr_solo
	4051_s250aq
	4070_aerofc
	4090_nanomind
	4100_tiltquadrotor
	4180_generic_quad_180
	4210_generic_quad_210
	4250_generic_quad_250
	4251_generic_racer_250
	4330_generic_quad_330
	4450_generic_quad_450
	4650_generic_quad_650
	4900_crazyflie
	
	# Multirotors
	5001_quad_+
	6001_hexa_x
	7001_hexa_+
	8001_octo_x
	9001_octo_+

	# Quadrotor Wide
	10015_tbs_discovery
	10016_3dr_iris
	10017_steadidrone_qu4d
	10018_tbs_endurance

	11001_hexa_cox

	12001_octo_cox
	12002_steadidrone_mavrik

	# VTOLs (Standard, Tailsitter, Tiltrotor)
	13000_generic_vtol_standard
	13001_caipirinha_vtol
	13002_firefly6
	13003_quad_tailsitter
	13004_quad+_tailsitter
	13005_vtol_AAERT_quad
	13006_vtol_standard_delta
	13007_vtol_AAVVT_quad
	13008_QuadRanger
	13009_vtol_spt_ranger
	13010_claire
	13012_convergence
	13013_deltaquad
	13200_generic_vtol_tailsitter

	# Tricopter
	14001_tri_y_yaw+
	14002_tri_y_yaw-

	# Coaxial helicopter
	15001_coax_heli
	
	# Helicopter
	16001_helicopter
	
	24001_dodeca_cox
	
	# Ground vehicle
	50000_generic_ground_vehicle
	50001_axialracing_ax10
	50002_traxxas_stampede_2wd
)
