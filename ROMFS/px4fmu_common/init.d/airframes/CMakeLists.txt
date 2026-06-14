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

# Airframes are filed by vehicle class in per-class subdirectories. A class
# directory is added to the ROMFS only when its CONFIG_AIRFRAMES_<class> option
# is set (see ROMFS/Kconfig); the option defaults follow the controller modules
# so existing configs ship the same set. The build flattens these directories
# back into /etc/init.d/airframes/ so the runtime layout is unchanged.
#
# Product/vendor-specific airframes that are not generally useful stay loose in
# this directory (no class subdir) and are intentionally not built. A board that
# needs one copies it into boards/<vendor>/<model>/init/airframes/.

set(airframe_classes
	"copter:AIRFRAMES_COPTER"
	"plane:AIRFRAMES_PLANE"
	"vtol:AIRFRAMES_VTOL"
	"rover:AIRFRAMES_ROVER"
	"uuv:AIRFRAMES_UUV"
	"spacecraft:AIRFRAMES_SPACECRAFT"
	"airship:AIRFRAMES_AIRSHIP"
	"balloon:AIRFRAMES_BALLOON"
	"simulation:AIRFRAMES_SIMULATION"
)

foreach(entry ${airframe_classes})
	string(REPLACE ":" ";" entry "${entry}")
	list(GET entry 0 class_dir)
	list(GET entry 1 class_option)

	if(CONFIG_${class_option})
		file(GLOB class_files RELATIVE ${CMAKE_CURRENT_LIST_DIR} ${class_dir}/*)
		if(class_files)
			px4_add_romfs_files(${class_files})
		endif()
	endif()
endforeach()
