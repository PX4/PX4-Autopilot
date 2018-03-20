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
	${CMAKE_CURRENT_LIST_DIR}/AAERTWF.main.mix
	${CMAKE_CURRENT_LIST_DIR}/AAVVTWFF.main.mix
	${CMAKE_CURRENT_LIST_DIR}/AERT.main.mix
	${CMAKE_CURRENT_LIST_DIR}/AETRFG.main.mix
	${CMAKE_CURRENT_LIST_DIR}/bebop.main.mix
	${CMAKE_CURRENT_LIST_DIR}/blade130.main.mix
	${CMAKE_CURRENT_LIST_DIR}/caipi.main.mix
	${CMAKE_CURRENT_LIST_DIR}/caipirinha_vtol.main.mix
	${CMAKE_CURRENT_LIST_DIR}/CCPM.main.mix
	${CMAKE_CURRENT_LIST_DIR}/claire.aux.mix
	${CMAKE_CURRENT_LIST_DIR}/claire.main.mix
	${CMAKE_CURRENT_LIST_DIR}/coax.main.mix
	${CMAKE_CURRENT_LIST_DIR}/delta.main.mix
	${CMAKE_CURRENT_LIST_DIR}/deltaquad.main.mix
	${CMAKE_CURRENT_LIST_DIR}/dodeca_bottom_cox.aux.mix
	${CMAKE_CURRENT_LIST_DIR}/dodeca_top_cox.main.mix
	${CMAKE_CURRENT_LIST_DIR}/firefly6.aux.mix
	${CMAKE_CURRENT_LIST_DIR}/firefly6.main.mix
	${CMAKE_CURRENT_LIST_DIR}/FMU_pass.mix
	${CMAKE_CURRENT_LIST_DIR}/fw_generic_wing.main.mix
	${CMAKE_CURRENT_LIST_DIR}/FX79.main.mix
	${CMAKE_CURRENT_LIST_DIR}/hexa_cox.main.mix
	${CMAKE_CURRENT_LIST_DIR}/hexa_+.main.mix
	${CMAKE_CURRENT_LIST_DIR}/hexa_x.main.mix
	${CMAKE_CURRENT_LIST_DIR}/IO_pass.main.mix
	${CMAKE_CURRENT_LIST_DIR}/mount.aux.mix
	${CMAKE_CURRENT_LIST_DIR}/mount_legs.aux.mix
	${CMAKE_CURRENT_LIST_DIR}/ocpoc_quad_x.main.mix
	${CMAKE_CURRENT_LIST_DIR}/octo_cox.main.mix
	${CMAKE_CURRENT_LIST_DIR}/octo_cox_w.main.mix
	${CMAKE_CURRENT_LIST_DIR}/octo_+.main.mix
	${CMAKE_CURRENT_LIST_DIR}/octo_x.main.mix
	${CMAKE_CURRENT_LIST_DIR}/pass.aux.mix
	${CMAKE_CURRENT_LIST_DIR}/phantom.main.mix
	${CMAKE_CURRENT_LIST_DIR}/quad_dc.main.mix
	${CMAKE_CURRENT_LIST_DIR}/quad_h.main.mix
	${CMAKE_CURRENT_LIST_DIR}/quad_+.main.mix
	${CMAKE_CURRENT_LIST_DIR}/quad_s250aq.main.mix
	${CMAKE_CURRENT_LIST_DIR}/quad_+_vtol.main.mix
	${CMAKE_CURRENT_LIST_DIR}/quad_w.main.mix
	${CMAKE_CURRENT_LIST_DIR}/quad_x_can.main.mix
	${CMAKE_CURRENT_LIST_DIR}/quad_x.main.mix
	${CMAKE_CURRENT_LIST_DIR}/quad_x_vtol.main.mix
	${CMAKE_CURRENT_LIST_DIR}/quad_x_vtol_sim.main.mix
	${CMAKE_CURRENT_LIST_DIR}/stampede.main.mix
	${CMAKE_CURRENT_LIST_DIR}/tri_y_yaw-.main.mix
	${CMAKE_CURRENT_LIST_DIR}/tri_y_yaw+.main.mix
	${CMAKE_CURRENT_LIST_DIR}/ugv_generic.main.mix
	${CMAKE_CURRENT_LIST_DIR}/Viper.main.mix
	${CMAKE_CURRENT_LIST_DIR}/vtol_AAERT.aux.mix
	${CMAKE_CURRENT_LIST_DIR}/vtol_AAVVT.aux.mix
	${CMAKE_CURRENT_LIST_DIR}/vtol_convergence.main.mix
	${CMAKE_CURRENT_LIST_DIR}/vtol_delta.aux.mix
	${CMAKE_CURRENT_LIST_DIR}/wingwing.main.mix
	${CMAKE_CURRENT_LIST_DIR}/zmr250.main.mix
	)

set(config_romfs_files_list ${config_romfs_files_list} PARENT_SCOPE)
