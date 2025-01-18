############################################################################
#
#   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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

function(message)
    if (NOT MESSAGE_QUIET)
        _message(${ARGN})
    endif()
endfunction()

set(POSIX_COMPATIBLE YES)
set(BUILD_SHARED_LIBS OFF)
set(BUILD_TESTING OFF)
set(CHECK_THREADS NO)
set(MESSAGE_QUIET ON)
set(ZENOH_DEBUG ${CONFIG_ZENOH_DEBUG})

px4_add_git_submodule(TARGET git_zenoh-pico PATH "zenoh-pico")
add_subdirectory(zenoh-pico)
unset(MESSAGE_QUIET)
add_dependencies(zenohpico git_zenoh-pico px4_platform)
target_compile_options(zenohpico PUBLIC -Wno-cast-align
			-Wno-narrowing
			-Wno-stringop-overflow
			-Wno-stringop-truncation
			-Wno-unused-result
			-DZ_BATCH_SIZE_RX=512
			-DZ_BATCH_SIZE_TX=512
			-DZ_FRAG_MAX_SIZE=1024)

target_compile_options(zenohpico PRIVATE -Wno-missing-prototypes)
if(CONFIG_PLATFORM_NUTTX)
    target_compile_options(zenohpico PRIVATE -DUNIX_NO_MULTICAST_IF)
endif()

if(CONFIG_ZENOH_SERIAL)
    target_compile_options(zenohpico PRIVATE -DZ_LINK_SERIAL)
endif()


px4_add_module(
		MODULE modules__zenoh
		MAIN zenoh
		SRCS
		zenoh.cpp
		zenoh_config.cpp
		publishers/zenoh_publisher.cpp
		subscribers/zenoh_subscriber.cpp
		MODULE_CONFIG
			module.yaml
		DEPENDS
			cdr
			uorb_msgs
			px4_work_queue
			zenohpico
			zenoh_topics
			git_zenoh-pico
		INCLUDES
			${PX4_BINARY_DIR}/msg
			zenoh-pico/include
			${CMAKE_CURRENT_LIST_DIR}
			${PX4_BINARY_DIR}/src/modules/zenoh/
		COMPILE_FLAGS
			-Wno-pointer-compare
			-Wno-cast-align
			-Wno-address-of-packed-member
			-Wno-double-promotion
			-Wno-unused
			-DZENOH_LINUX
			-D_Bool=int8_t
)
