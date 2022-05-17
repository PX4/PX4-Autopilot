############################################################################
#
#   Copyright (c) 2017 PX4 Development Team. All rights reserved.
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
add_library(sagetech_sdk_public STATIC IMPORTED GLOBAL)

px4_add_module(
	MODULE drivers__transponder__sagetech_mxs
	MAIN mxs
	COMPILE_FLAGS
	INCLUDES
		SagetechSDK
	SRCS
		SagetechSDK/sgDecodeMSR.c
		SagetechSDK/sgDecodeSVR.c
		SagetechSDK/sgEncodeGPS.c
		SagetechSDK/appendChecksum.c
		SagetechSDK/calcChecksum.c
		SagetechSDK/charArray2Buf.c
		SagetechSDK/double2Buf.c
		SagetechSDK/float2Buf.c
		SagetechSDK/icao2Buf.c
		SagetechSDK/int242Buf.c
		SagetechSDK/latLon2Buf.c
		SagetechSDK/target.c
		SagetechSDK/toAlt.c
		SagetechSDK/toDist.c
		SagetechSDK/toDouble.c
		SagetechSDK/toFloat.c
		SagetechSDK/toGS.c
		SagetechSDK/toHeading.c
		SagetechSDK/toHeading2.c
		SagetechSDK/toIcao.c
		SagetechSDK/toInt16.c
		SagetechSDK/toInt32.c
		SagetechSDK/toLatLon.c
		SagetechSDK/toTOA.c
		SagetechSDK/toUint16.c
		SagetechSDK/toUint24.c
		SagetechSDK/toUint32.c
		SagetechSDK/toVel.c
		SagetechSDK/uint162Buf.c
		SagetechSDK/uint242Buf.c
		SagetechSDK/uint322Buf.c
		MXS.cpp
		MXS.hpp
		MXS_main.cpp
	MODULE_CONFIG
		module.yaml
	)