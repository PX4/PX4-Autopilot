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

list(APPEND config_romfs_files_list
	${CMAKE_CURRENT_LIST_DIR}/1000_rc_fw_easystar.hil
	${CMAKE_CURRENT_LIST_DIR}/1001_rc_quad_x.hil
	${CMAKE_CURRENT_LIST_DIR}/2100_standard_plane
	${CMAKE_CURRENT_LIST_DIR}/2105_maja
	${CMAKE_CURRENT_LIST_DIR}/2106_albatross
	${CMAKE_CURRENT_LIST_DIR}/24001_dodeca_cox
	${CMAKE_CURRENT_LIST_DIR}/3000_generic_wing
	${CMAKE_CURRENT_LIST_DIR}/3030_io_camflyer
	${CMAKE_CURRENT_LIST_DIR}/3031_phantom
	${CMAKE_CURRENT_LIST_DIR}/3032_skywalker_x5
	${CMAKE_CURRENT_LIST_DIR}/3033_wingwing
	${CMAKE_CURRENT_LIST_DIR}/3034_fx79
	${CMAKE_CURRENT_LIST_DIR}/3035_viper
	${CMAKE_CURRENT_LIST_DIR}/3036_pigeon
	${CMAKE_CURRENT_LIST_DIR}/3037_parrot_disco_mod
	${CMAKE_CURRENT_LIST_DIR}/3100_tbs_caipirinha
	${CMAKE_CURRENT_LIST_DIR}/4001_quad_x
	${CMAKE_CURRENT_LIST_DIR}/4002_quad_x_mount
	${CMAKE_CURRENT_LIST_DIR}/4003_qavr5
	${CMAKE_CURRENT_LIST_DIR}/4004_H4_680mm
	${CMAKE_CURRENT_LIST_DIR}/4009_qav250
	${CMAKE_CURRENT_LIST_DIR}/4010_dji_f330
	${CMAKE_CURRENT_LIST_DIR}/4011_dji_f450
	${CMAKE_CURRENT_LIST_DIR}/4012_quad_x_can
	${CMAKE_CURRENT_LIST_DIR}/4013_bebop
	${CMAKE_CURRENT_LIST_DIR}/4020_hk_micro_pcb
	${CMAKE_CURRENT_LIST_DIR}/4030_3dr_solo
	${CMAKE_CURRENT_LIST_DIR}/4031_3dr_quad
	${CMAKE_CURRENT_LIST_DIR}/4040_reaper
	${CMAKE_CURRENT_LIST_DIR}/4050_generic_250
	${CMAKE_CURRENT_LIST_DIR}/4051_s250aq
	${CMAKE_CURRENT_LIST_DIR}/4060_dji_matrice_100
	${CMAKE_CURRENT_LIST_DIR}/4070_aerofc
	${CMAKE_CURRENT_LIST_DIR}/4080_zmr250
	${CMAKE_CURRENT_LIST_DIR}/4090_nanomind
	${CMAKE_CURRENT_LIST_DIR}/4900_crazyflie
	${CMAKE_CURRENT_LIST_DIR}/5001_quad_+
	${CMAKE_CURRENT_LIST_DIR}/6001_hexa_x
	${CMAKE_CURRENT_LIST_DIR}/7001_hexa_+
	${CMAKE_CURRENT_LIST_DIR}/8001_octo_x
	${CMAKE_CURRENT_LIST_DIR}/9001_octo_+
	${CMAKE_CURRENT_LIST_DIR}/10015_tbs_discovery
	${CMAKE_CURRENT_LIST_DIR}/10016_3dr_iris
	${CMAKE_CURRENT_LIST_DIR}/10017_steadidrone_qu4d
	${CMAKE_CURRENT_LIST_DIR}/10018_tbs_endurance
	${CMAKE_CURRENT_LIST_DIR}/11001_hexa_cox
	${CMAKE_CURRENT_LIST_DIR}/12001_octo_cox
	${CMAKE_CURRENT_LIST_DIR}/12002_steadidrone_mavrik
	${CMAKE_CURRENT_LIST_DIR}/13000_generic_vtol_standard
	${CMAKE_CURRENT_LIST_DIR}/13001_caipirinha_vtol
	${CMAKE_CURRENT_LIST_DIR}/13002_firefly6
	${CMAKE_CURRENT_LIST_DIR}/13003_quad_tailsitter
	${CMAKE_CURRENT_LIST_DIR}/13004_quad+_tailsitter
	${CMAKE_CURRENT_LIST_DIR}/13005_vtol_AAERT_quad
	${CMAKE_CURRENT_LIST_DIR}/13006_vtol_standard_delta
	${CMAKE_CURRENT_LIST_DIR}/13007_vtol_AAVVT_quad
	${CMAKE_CURRENT_LIST_DIR}/13008_QuadRanger
	${CMAKE_CURRENT_LIST_DIR}/13009_vtol_spt_ranger
	${CMAKE_CURRENT_LIST_DIR}/13010_claire
	${CMAKE_CURRENT_LIST_DIR}/13012_convergence
	${CMAKE_CURRENT_LIST_DIR}/13013_deltaquad
	${CMAKE_CURRENT_LIST_DIR}/14001_tri_y_yaw+
	${CMAKE_CURRENT_LIST_DIR}/14002_tri_y_yaw-
	${CMAKE_CURRENT_LIST_DIR}/15001_coax_heli
	${CMAKE_CURRENT_LIST_DIR}/16001_helicopter
	${CMAKE_CURRENT_LIST_DIR}/50000_generic_ground_vehicle
	${CMAKE_CURRENT_LIST_DIR}/50001_axialracing_ax10
	${CMAKE_CURRENT_LIST_DIR}/50002_traxxas_stampede_2wd	
	${CMAKE_CURRENT_LIST_DIR}/rc.axialracing_ax10_apps
	${CMAKE_CURRENT_LIST_DIR}/rc.axialracing_ax10_defaults
	${CMAKE_CURRENT_LIST_DIR}/rc.fw_apps
	${CMAKE_CURRENT_LIST_DIR}/rc.fw_defaults
	${CMAKE_CURRENT_LIST_DIR}/rc.interface
	${CMAKE_CURRENT_LIST_DIR}/rc.io
	${CMAKE_CURRENT_LIST_DIR}/rc.logging
	${CMAKE_CURRENT_LIST_DIR}/rc.mc_apps
	${CMAKE_CURRENT_LIST_DIR}/rc.mc_defaults
	${CMAKE_CURRENT_LIST_DIR}/rcS
	${CMAKE_CURRENT_LIST_DIR}/rc.sensors
	${CMAKE_CURRENT_LIST_DIR}/rc.ugv_apps
	${CMAKE_CURRENT_LIST_DIR}/rc.ugv_defaults
	${CMAKE_CURRENT_LIST_DIR}/rc.vtol_apps
	${CMAKE_CURRENT_LIST_DIR}/rc.vtol_defaults	
	)

set(config_romfs_files_list ${config_romfs_files_list} PARENT_SCOPE)
