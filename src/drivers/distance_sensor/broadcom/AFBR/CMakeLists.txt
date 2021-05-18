############################################################################
#
#   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

add_library(afbrs50_m4_fpu STATIC IMPORTED GLOBAL)
set_property(TARGET afbrs50_m4_fpu PROPERTY IMPORTED_LOCATION ${CMAKE_CURRENT_SOURCE_DIR}/Lib/libafbrs50_m4_fpu.a)

px4_add_module(
	MODULE drivers__distance_sensor__afbrs50
	MAIN afbrs50
	COMPILE_FLAGS
		-Wno-error
	STACK_MAIN
		8192
	INCLUDES
		Inc
		API/Inc
		Core/Inc
		Drivers/CMSIS/Include
	SRCS
		AFBRS50.cpp
		AFBRS50.hpp
		afbrs50_main.cpp
		API/Src/irq.c
		API/Src/s2pi.c
		API/Src/timer.c
	DEPENDS
		px4_work_queue
		drivers_rangefinder
		afbrs50_m4_fpu
	)

target_link_libraries(afbrs50_m4_fpu INTERFACE drivers__distance_sensor__afbrs50)
