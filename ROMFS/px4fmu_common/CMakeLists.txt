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

set(config_romfs_files_list
	init.d/1000_rc_fw_easystar.hil
	init.d/1001_rc_quad_x.hil
	init.d/2100_standard_plane
	init.d/2105_maja
	init.d/2106_albatross
	init.d/24001_dodeca_cox
	init.d/3000_generic_wing
	init.d/3030_io_camflyer
	init.d/3031_phantom
	init.d/3032_skywalker_x5
	init.d/3033_wingwing
	init.d/3034_fx79
	init.d/3035_viper
	init.d/3036_pigeon
	init.d/3037_parrot_disco_mod
	init.d/3100_tbs_caipirinha
	init.d/4001_quad_x
	init.d/4002_quad_x_mount
	init.d/4003_qavr5
	init.d/4004_H4_680mm
	init.d/4009_qav250
	init.d/4010_dji_f330
	init.d/4011_dji_f450
	init.d/4012_quad_x_can
	init.d/4013_bebop
	init.d/4020_hk_micro_pcb
	init.d/4030_3dr_solo
	init.d/4031_3dr_quad
	init.d/4040_reaper
	init.d/4050_generic_250
	init.d/4051_s250aq
	init.d/4060_dji_matrice_100
	init.d/4070_aerofc
	init.d/4080_zmr250
	init.d/4090_nanomind
	init.d/4900_crazyflie
	init.d/5001_quad_+
	init.d/6001_hexa_x
	init.d/7001_hexa_+
	init.d/8001_octo_x
	init.d/9001_octo_+
	init.d/10015_tbs_discovery
	init.d/10016_3dr_iris
	init.d/10017_steadidrone_qu4d
	init.d/10018_tbs_endurance
	init.d/11001_hexa_cox
	init.d/12001_octo_cox
	init.d/12002_steadidrone_mavrik
	init.d/13000_generic_vtol_standard
	init.d/13001_caipirinha_vtol
	init.d/13002_firefly6
	init.d/13003_quad_tailsitter
	init.d/13004_quad+_tailsitter
	init.d/13005_vtol_AAERT_quad
	init.d/13006_vtol_standard_delta
	init.d/13007_vtol_AAVVT_quad
	init.d/13008_QuadRanger
	init.d/13009_vtol_spt_ranger
	init.d/13010_claire
	init.d/13012_convergence
	init.d/13013_deltaquad
	init.d/14001_tri_y_yaw+
	init.d/14002_tri_y_yaw-
	init.d/15001_coax_heli
	init.d/16001_helicopter
	init.d/50000_generic_ground_vehicle
	init.d/50001_axialracing_ax10
	init.d/50002_traxxas_stampede_2wd	
	init.d/rc.axialracing_ax10_apps
	init.d/rc.axialracing_ax10_defaults
	init.d/rc.fw_apps
	init.d/rc.fw_defaults
	init.d/rc.interface
	init.d/rc.io
	init.d/rc.logging
	init.d/rc.mc_apps
	init.d/rc.mc_defaults
	init.d/rcS
	init.d/rc.sensors
	init.d/rc.ugv_apps
	init.d/rc.ugv_defaults
	init.d/rc.vtol_apps
	init.d/rc.vtol_defaults
	mixers/AAERTWF.main.mix
	mixers/AAVVTWFF.main.mix
	mixers/AERT.main.mix
	mixers/AETRFG.main.mix
	mixers/bebop.main.mix
	mixers/blade130.main.mix
	mixers/caipi.main.mix
	mixers/caipirinha_vtol.main.mix
	mixers/CCPM.main.mix
	mixers/claire.aux.mix
	mixers/claire.main.mix
	mixers/coax.main.mix
	mixers/delta.main.mix
	mixers/deltaquad.main.mix
	mixers/dodeca_bottom_cox.aux.mix
	mixers/dodeca_top_cox.main.mix
	mixers/firefly6.aux.mix
	mixers/firefly6.main.mix
	mixers/FMU_pass.mix
	mixers/fw_generic_wing.main.mix
	mixers/FX79.main.mix
	mixers/hexa_cox.main.mix
	mixers/hexa_+.main.mix
	mixers/hexa_x.main.mix
	mixers/IO_pass.main.mix
	mixers/mount.aux.mix
	mixers/mount_legs.aux.mix
	mixers/ocpoc_quad_x.main.mix
	mixers/octo_cox.main.mix
	mixers/octo_cox_w.main.mix
	mixers/octo_+.main.mix
	mixers/octo_x.main.mix
	mixers/pass.aux.mix
	mixers/phantom.main.mix
	mixers/quad_dc.main.mix
	mixers/quad_h.main.mix
	mixers/quad_+.main.mix
	mixers/quad_s250aq.main.mix
	mixers/quad_+_vtol.main.mix
	mixers/quad_w.main.mix
	mixers/quad_x_can.main.mix
	mixers/quad_x.main.mix
	mixers/quad_x_vtol.main.mix
	mixers/quad_x_vtol_sim.main.mix
	mixers/stampede.main.mix
	mixers/tri_y_yaw-.main.mix
	mixers/tri_y_yaw+.main.mix
	mixers/ugv_generic.main.mix
	mixers/Viper.main.mix
	mixers/vtol_AAERT.aux.mix
	mixers/vtol_AAVVT.aux.mix
	mixers/vtol_convergence.main.mix
	mixers/vtol_delta.aux.mix
	mixers/wingwing.main.mix
	mixers/zmr250.main.mix
	)

set(config_romfs_files_list ${config_romfs_files_list} PARENT_SCOPE)
