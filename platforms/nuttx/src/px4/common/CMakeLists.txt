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

set(PX4_KERNEL_LAYER px4_layer)

# skip for px4_layer support on an IO board
if(NOT PX4_BOARD MATCHES "io-v2")
	# Kernel side & nuttx flat build common sources
	set(KERNEL_SRCS
		board_crashdump.c
		board_dma_alloc.c
		board_fat_dma_alloc.c
		console_buffer.cpp
		cpuload.cpp
		gpio.c
		print_load.cpp
		tasks.cpp
		px4_nuttx_impl.cpp
		px4_init.cpp
		px4_manifest.cpp
		px4_mtd.cpp
		px4_24xxxx_mtd.c
		px4_crypto.cpp
	)

	# Kernel side & nuttx flat build common libraries
	set(KERNEL_LIBS
		arch_board_reset
		arch_board_critmon
		arch_hrt
		arch_version
		nuttx_xx
		nuttx_sched
	)

if (NOT DEFINED CONFIG_BUILD_FLAT AND "${PX4_PLATFORM}" MATCHES "nuttx")
	# Build two layers, one for user side (px4_layer) and one for kernel (px4_kernel_layer)
	set(PX4_USER_LAYER px4_layer)
	set(PX4_KERNEL_LAYER px4_kernel_layer)

	list(APPEND KERNEL_LIBS
		nuttx_kc
		nuttx_karch
		nuttx_kmm
	)

	add_library(${PX4_USER_LAYER}
		tasks.cpp
		console_buffer_usr.cpp
		usr_mcu_version.cpp
		cdc_acm_check.cpp
		${PX4_SOURCE_DIR}/platforms/posix/src/px4/common/print_load.cpp
		${PX4_SOURCE_DIR}/platforms/posix/src/px4/common/cpuload.cpp
	)

	target_link_libraries(${PX4_USER_LAYER}
		PRIVATE
			m
			nuttx_c
			nuttx_xx
			nuttx_mm
	)

	add_dependencies(${PX4_USER_LAYER} prebuild_targets)
else()
	# Build the px4 layer for nuttx flat build
	list(APPEND KERNEL_LIBS
		nuttx_c
		nuttx_arch
		nuttx_mm
		)

	list(APPEND KERNEL_SRCS
		cdc_acm_check.cpp
	)
endif()

add_library(${PX4_KERNEL_LAYER}
	${KERNEL_SRCS}
)

target_link_libraries(${PX4_KERNEL_LAYER}
	PRIVATE
		${KERNEL_LIBS}
)

target_compile_options(${PX4_KERNEL_LAYER} PRIVATE -D__KERNEL__)

else()
	# Build the px4 layer for posix
        add_library(${PX4_KERNEL_LAYER} ${PX4_SOURCE_DIR}/platforms/common/empty.c)
endif()

add_dependencies(${PX4_KERNEL_LAYER} prebuild_targets)

add_subdirectory(gpio)
add_subdirectory(srgbled)

if (DEFINED PX4_CRYPTO)
	add_library(px4_random nuttx_random.c)
	add_dependencies(px4_random nuttx_context)
	target_link_libraries(px4_random PRIVATE nuttx_crypto)

	target_link_libraries(${PX4_KERNEL_LAYER}
		PUBLIC
			crypto_backend
	)
endif()
