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

set(mixer_file_names
	AAERTWF.main.mix
	AAVVTWFF.main.mix
	AERT.main.mix
	AET.main.mix
	AETR.main.mix
	caipi.main.mix
	caipirinha_vtol.main.mix
	CCPM.main.mix
	claire.aux.mix
	claire.main.mix
	CMakeLists.txt
	coax.main.mix
	delta.main.mix
	easystar.main.mix
	firefly6.aux.mix
	firefly6.main.mix
	FMU_pass.mix
	FX79.main.mix
	hexa_cox.main.mix
	hexa_+.main.mix
	hexa_x.main.mix
	IO_pass.main.mix
	IO_pass.mix
	octo_cox.main.mix
	octo_cox_w.main.mix
	octo_+.main.mix
	octo_x.main.mix
	pass.aux.mix
	phantom.main.mix
	Q.main.mix
	quad_h.main.mix
	quad_+.main.mix
	quad_+_vtol.main.mix
	quad_w.main.mix
	quad_x.main.mix
	quad_x_vtol.main.mix
	README.md
	RET.main.mix
	skywalker.main.mix
	tri_y_yaw-.main.mix
	tri_y_yaw+.main.mix
	Viper.main.mix
	vtol_AAERT.aux.mix
	vtol_AAVVT.aux.mix
	vtol_delta.aux.mix
	vtol_quad_x.main.mix
	wingwing.main.mix
	X5.main.mix
	mount_legs.aux.mix
)

# Get absolute paths
set(mixer_files)
foreach(mixer_file ${mixer_file_names})
	list(APPEND mixer_files ${CMAKE_CURRENT_SOURCE_DIR}/${mixer_file})
endforeach()

set(mixer_directory ${ROOTFS_DIRECTORY}/etc/mixers)

add_custom_target(mixer_dir
	COMMAND ${CMAKE_COMMAND} -E make_directory ${mixer_directory}
)

# cmake -E copy with version < 3.5 does not support copying multiple files yet, therefore use cp.
add_custom_target(mixer
	COMMAND cp ${mixer_files} ${mixer_directory}
	DEPENDS mixer_dir
)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
