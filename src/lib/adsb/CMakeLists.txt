############################################################################
#
#   Copyright (c) 2022-2026 PX4 Development Team. All rights reserved.
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

set(adsb_srcs
	AdsbConflict.cpp
	ConflictTracker.cpp
	DaaActionPolicy.cpp
	DaaEncodedId.cpp
)

if(CONFIG_NAVIGATOR_ADSB_F3442)
	list(APPEND adsb_srcs f3442_standard/DaaF3442.cpp)
else()
	list(APPEND adsb_srcs crosstrack_standard/DaaCrosstrack.cpp)
endif()

px4_add_library(adsb ${adsb_srcs})
set_property(GLOBAL APPEND PROPERTY PX4_MODULE_CONFIG_FILES ${CMAKE_CURRENT_SOURCE_DIR}/parameters.yaml)

target_link_libraries(adsb PUBLIC geo)


if(CONFIG_NAVIGATOR_ADSB)
	px4_add_functional_gtest(SRC AdsbConflictTest.cpp LINKLIBS adsb)
	px4_add_functional_gtest(SRC ConflictTrackerTest.cpp LINKLIBS adsb)
	px4_add_unit_gtest(SRC DaaActionPolicyTest.cpp LINKLIBS adsb)
	px4_add_unit_gtest(SRC DaaEncodedIdTest.cpp LINKLIBS adsb)

	if(CONFIG_NAVIGATOR_ADSB_F3442)
		px4_add_functional_gtest(SRC f3442_standard/DaaF3442Test.cpp LINKLIBS adsb)
	else()
		px4_add_functional_gtest(SRC crosstrack_standard/DaaCrosstrackTest.cpp LINKLIBS adsb)
	endif()
endif()
