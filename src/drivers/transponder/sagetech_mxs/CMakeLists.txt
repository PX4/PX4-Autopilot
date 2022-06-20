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

px4_add_module(
	MODULE drivers__transponder__sagetech_mxs
	MAIN sagetech_mxs
	COMPILE_FLAGS
		#-DDEBUG_BUILD   # uncomment for PX4_DEBUG output
		#-O0             # uncomment when debugging
	SRCS
		sg_sdk/appendChecksum.c
		sg_sdk/float2Buf.c
		sg_sdk/sgDecodeFlightId.c
		sg_sdk/sgDecodeSVR.c
		sg_sdk/sgEncodeGPS.c
		sg_sdk/sgEncodeTargetReq.c
		sg_sdk/target.c
		sg_sdk/toGS.c
		sg_sdk/toIcao.c
		sg_sdk/toLatLon.c
		sg_sdk/toUint32.c
		sg_sdk/uint322Buf.c
		sg_sdk/calcChecksum.c
		sg_sdk/icao2Buf.c
		sg_sdk/sgDecodeInstall.c
		sg_sdk/sgEncodeDataReq.c
		sg_sdk/sgEncodeInstall.c
		sg_sdk/toHeading2.c
		sg_sdk/toInt16.c
		sg_sdk/toTOA.c
		sg_sdk/toVel.c
		sg_sdk/charArray2Buf.c
		sg_sdk/sgDecodeAck.c
		sg_sdk/sgDecodeMSR.c
		sg_sdk/sgEncodeFlightId.c
		sg_sdk/sgEncodeOperating.c
		sg_sdk/toAlt.c
		sg_sdk/toHeading.c
		sg_sdk/toInt32.c
		sg_sdk/toUint16.c
		sg_sdk/uint162Buf.c
		sg_sdk/sagetech_mxs.h
		sg_sdk/sg.h
		sg_sdk/target.h
		sg_sdk/sgUtil.h
		SagetechMXS.cpp
		SagetechMXS.hpp
	DEPENDS
		px4_work_queue
	MODULE_CONFIG
		module.yaml
	)
