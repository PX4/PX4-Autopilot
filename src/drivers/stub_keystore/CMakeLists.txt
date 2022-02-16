
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

px4_add_library(keystore_backend stub_keystore.c)

target_include_directories(keystore_backend PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Parse keyfile locations from boardconfig

# If the key file path is defined in environment variable, use it from there
# else, if it is hardcoded in CONFIG_PUBLIC_KEYx, use that

file(STRINGS ${BOARD_CONFIG} ConfigContents)
foreach(NameAndValue ${ConfigContents})
# Strip leading spaces
    string(REGEX REPLACE "^[ ]+" "" NameAndValue ${NameAndValue})

    # Find variable name
    string(REGEX MATCH "^CONFIG_PUBLIC_KEY[^=]+" Name ${NameAndValue})

	if(Name)
	  string(REPLACE "${Name}=" "" Value ${NameAndValue})
	  string(REPLACE "\"" "" Value ${Value})

	  # Strip CONFIG_
	  string(REPLACE "CONFIG_" "" Name ${Name})

	  # Get the value from env variable, if set
	  if(DEFINED ENV{${Name}})
		set(Value $ENV{${Name}})
	  endif()

	  if(NOT Value STREQUAL "")
		add_definitions(-D${Name}=${Value})
	  endif()
	endif()
endforeach()
