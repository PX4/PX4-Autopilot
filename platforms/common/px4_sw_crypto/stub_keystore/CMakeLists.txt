
############################################################################
#
#   Copyright (c) 2021 Technology Innoavation Institute. All rights reserved.
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

px4_add_library(arch_keystore stub_keystore.c)

target_include_directories(arch_keystore PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if(DEFINED ENV{PUBLIC_KEY0})
	add_definitions(-DPUBLIC_KEY0=$ENV{PUBLIC_KEY0})
endif()

if(DEFINED ENV{PUBLIC_KEY1})
	add_definitions(-DPUBLIC_KEY1=$ENV{PUBLIC_KEY1})
endif()

if(DEFINED ENV{PUBLIC_KEY2})
	add_definitions(-DPUBLIC_KEY2=$ENV{PUBLIC_KEY2})
endif()

if(DEFINED ENV{PUBLIC_KEY3})
	add_definitions(-DPUBLIC_KEY3=$ENV{PUBLIC_KEY3})
endif()

if(DEFINED ENV{PUBLIC_KEY4})
	add_definitions(-DPUBLIC_KEY4=$ENV{PUBLIC_KEY4})
endif()

if(DEFINED ENV{PUBLIC_KEY5})
	add_definitions(-DPUBLIC_KEY5=$ENV{PUBLIC_KEY5})
endif()

if(DEFINED ENV{PUBLIC_KEY6})
	add_definitions(-DPUBLIC_KEY6=$ENV{PUBLIC_KEY6})
endif()

if(DEFINED ENV{PUBLIC_KEY7})
	add_definitions(-DPUBLIC_KEY7=$ENV{PUBLIC_KEY7})
endif()
