############################################################################
#
#   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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

# Support IN_LIST if() operator
cmake_policy(SET CMP0057 NEW)

set_property(GLOBAL PROPERTY AIRFRAME_PARAMETER_FILES
	1000.param
	1001.param
	1002.param
	1100.param
	2100.param
	2105.param
	2106.param
	2200.param
	3030.param
	3031.param
	3032.param
	3033.param
	3034.param
	3036.param
	3037.param
	3100.param
	4002.param
	4003.param
	4004.param
	4009.param
	4010.param
	4011.param
	4012.param
	4014.param
	4015.param
	4016.param
	4020.param
	4030.param
	4031.param
	4040.param
	4041.param
	4050.param
	4051.param
	4052.param
	4053.param
	4060.param
	4070.param
	4071.param
	4072.param
	4080.param
	4090.param
	4100.param
	4250.param
	4500.param
	4900.param
	6002.param
	10015.param
	10016.param
	10017.param
	10018.param
	12002.param
	13000.param
	13001.param
	13002.param
	13003.param
	13004.param
	13005.param
	13006.param
	13007.param
	13008.param
	13009.param
	13010.param
	13013.param
	13014.param
	13050.param
	13200.param
	15001.param
	16001.param
	24001.param
	50000.param
	50001.param
	50002.param
	50003.param
)