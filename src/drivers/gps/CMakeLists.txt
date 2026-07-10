############################################################################
#
#   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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

px4_add_git_submodule(TARGET git_gps_devices PATH "devices")

set(SRCS
	gps.cpp
	devices/src/gps_helper.cpp
)

if(CONFIG_GPS_UBX)
	list(APPEND SRCS devices/src/ubx.cpp)
endif()

if(CONFIG_GPS_MTK)
	list(APPEND SRCS devices/src/mtk.cpp)
endif()

if(CONFIG_GPS_ASHTECH)
	list(APPEND SRCS devices/src/ashtech.cpp)
endif()

if(CONFIG_GPS_EMLIDREACH)
	list(APPEND SRCS devices/src/emlid_reach.cpp)
endif()

if(CONFIG_GPS_FEMTOMES)
	list(APPEND SRCS devices/src/femtomes.cpp)
endif()

if(CONFIG_GPS_NMEA)
	list(APPEND SRCS devices/src/nmea.cpp devices/src/unicore.cpp)
endif()

# RTCM parsing (RTK base station corrections) shared by these receivers
if(CONFIG_GPS_UBX OR CONFIG_GPS_ASHTECH OR CONFIG_GPS_FEMTOMES OR CONFIG_GPS_NMEA)
	list(APPEND SRCS devices/src/rtcm.cpp)
endif()

# CRC helpers shared by Femtomes and Unicore (NMEA)
if(CONFIG_GPS_FEMTOMES OR CONFIG_GPS_NMEA)
	list(APPEND SRCS devices/src/crc.cpp)
endif()

px4_add_module(
	MODULE drivers__gps
	MAIN gps
	COMPILE_FLAGS
		-Wno-stringop-overflow # due to https://gcc.gnu.org/bugzilla/show_bug.cgi?id=91707
	SRCS
		${SRCS}
	MODULE_CONFIG
		module.yaml
		params.yaml
	DEPENDS
		git_gps_devices
		gnss
	)
