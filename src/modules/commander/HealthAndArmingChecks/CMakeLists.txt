############################################################################
#
#   Copyright (c) 2022 PX4 Development Team. All rights reserved.
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

px4_add_library(health_and_arming_checks
	Common.cpp
	HealthAndArmingChecks.cpp

	checks/accelerometerCheck.cpp
	checks/airspeedCheck.cpp
	checks/armPermissionCheck.cpp
	checks/baroCheck.cpp
	checks/batteryCheck.cpp
	checks/cpuResourceCheck.cpp
	checks/distanceSensorChecks.cpp
	checks/escCheck.cpp
	checks/estimatorCheck.cpp
	checks/failureDetectorCheck.cpp
	checks/flightTimeCheck.cpp
	checks/geofenceCheck.cpp
	checks/gyroCheck.cpp
	checks/homePositionCheck.cpp
	checks/imuConsistencyCheck.cpp
	checks/loggerCheck.cpp
	checks/magnetometerCheck.cpp
	checks/manualControlCheck.cpp
	checks/missionCheck.cpp
	checks/modeCheck.cpp
	checks/offboardCheck.cpp
	checks/openDroneIDCheck.cpp
	checks/parachuteCheck.cpp
	checks/powerCheck.cpp
	checks/rcAndDataLinkCheck.cpp
	checks/rcCalibrationCheck.cpp
	checks/sdcardCheck.cpp
	checks/systemCheck.cpp
	checks/vtolCheck.cpp
	checks/windCheck.cpp

	checks/externalChecks.cpp
)
add_dependencies(health_and_arming_checks mode_util)

px4_add_functional_gtest(SRC HealthAndArmingChecksTest.cpp
	LINKLIBS health_and_arming_checks mode_util
	)
