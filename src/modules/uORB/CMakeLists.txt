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

# this includes the generated topics directory
include_directories(
	${CMAKE_CURRENT_BINARY_DIR}
	)

set(SRCS
	objects_common.cpp
	uORBUtils.cpp
	uORB.cpp
	uORBMain.cpp
	Publication.cpp
	Subscription.cpp
	)

if(${OS} STREQUAL "nuttx")
	list(APPEND SRCS
		uORBDevices_nuttx.cpp
		uORBTest_UnitTest.cpp
		uORBManager_nuttx.cpp
		)
elseif(${OS} STREQUAL "posix")
	list(APPEND SRCS
		uORBDevices_posix.cpp
		uORBManager_posix.cpp
		uORBTest_UnitTest.cpp
		)
elseif(${OS} STREQUAL "posix-arm")
	list(APPEND SRCS
		uORBDevices_posix.cpp
		uORBManager_posix.cpp
		uORBTest_UnitTest.cpp
		)
elseif(${OS} STREQUAL "qurt")
	list(APPEND SRCS
		uORBDevices_posix.cpp
		uORBManager_posix.cpp
		)
endif()

px4_add_module(
	MODULE modules__uORB
	MAIN uorb
	STACK 2048
	COMPILE_FLAGS
		-Os
	SRCS ${SRCS}
	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
