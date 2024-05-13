############################################################################
#
#   Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

px4_add_library(ActuatorEffectiveness
	ActuatorEffectiveness.cpp
	ActuatorEffectiveness.hpp
	ActuatorEffectivenessUUV.cpp
	ActuatorEffectivenessUUV.hpp
	ActuatorEffectivenessControlSurfaces.cpp
	ActuatorEffectivenessControlSurfaces.hpp
	ActuatorEffectivenessCustom.cpp
	ActuatorEffectivenessCustom.hpp
	ActuatorEffectivenessFixedWing.cpp
	ActuatorEffectivenessFixedWing.hpp
	ActuatorEffectivenessHelicopter.cpp
	ActuatorEffectivenessHelicopter.hpp
	ActuatorEffectivenessHelicopterCoaxial.cpp
	ActuatorEffectivenessHelicopterCoaxial.hpp
	ActuatorEffectivenessMCTilt.cpp
	ActuatorEffectivenessMCTilt.hpp
	ActuatorEffectivenessMultirotor.cpp
	ActuatorEffectivenessMultirotor.hpp
	ActuatorEffectivenessTilts.cpp
	ActuatorEffectivenessTilts.hpp
	ActuatorEffectivenessRotors.cpp
	ActuatorEffectivenessRotors.hpp
	ActuatorEffectivenessThrusters.cpp
	ActuatorEffectivenessThrusters.hpp
	ActuatorEffectivenessSpacecraft.cpp
	ActuatorEffectivenessSpacecraft.hpp
	ActuatorEffectivenessStandardVTOL.cpp
	ActuatorEffectivenessStandardVTOL.hpp
	ActuatorEffectivenessTiltrotorVTOL.cpp
	ActuatorEffectivenessTiltrotorVTOL.hpp
	ActuatorEffectivenessTailsitterVTOL.cpp
	ActuatorEffectivenessTailsitterVTOL.hpp
	ActuatorEffectivenessRoverAckermann.hpp
	ActuatorEffectivenessRoverAckermann.cpp
)

target_compile_options(ActuatorEffectiveness PRIVATE ${MAX_CUSTOM_OPT_LEVEL})
target_include_directories(ActuatorEffectiveness PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(ActuatorEffectiveness
	PRIVATE
		mathlib
)

px4_add_functional_gtest(SRC ActuatorEffectivenessHelicopterTest.cpp LINKLIBS ActuatorEffectiveness)
px4_add_functional_gtest(SRC ActuatorEffectivenessRotorsTest.cpp LINKLIBS ActuatorEffectiveness)
px4_add_functional_gtest(SRC ActuatorEffectivenessThrustersTest.cpp LINKLIBS ActuatorEffectiveness)