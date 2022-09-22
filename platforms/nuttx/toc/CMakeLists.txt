############################################################################
#
#   Copyright (c) 2022 Technology Innovation Institute. All rights reserved.
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

add_executable(toc
	fw_image.c
)

set(TOC_NAME ${PX4_BINARY_DIR}/toc.elf)
set_target_properties(toc PROPERTIES OUTPUT_NAME ${TOC_NAME})
target_compile_options(toc PRIVATE -DPX4_UNSIGNED_FIRMWARE=${PX4_BINARY_OUTPUT})
set_source_files_properties(fw_image.c PROPERTIES OBJECT_DEPENDS px4_binary)

add_library(board_toc
	${PX4_BOARD_DIR}/src/toc.c # The board specific ToC file
)
add_dependencies(board_toc nuttx_context)

target_link_libraries(toc PRIVATE

	-nostartfiles
	-nodefaultlibs
	-nostdlib

	-Wl,--script=${NUTTX_CONFIG_DIR}/scripts/${SCRIPT_PREFIX}toc.ld
	-Wl,--warn-common
	-Wl,-Map=${PX4_BINARY_DIR}/toc.map
)

if(NOT USE_LD_GOLD)
	target_link_libraries(toc PRIVATE -Wl,--print-memory-usage)
endif()

target_link_libraries(toc PRIVATE board_toc)
target_link_libraries(drivers_board PRIVATE board_toc)

set(TOC_BINARY_OUTPUT ${PX4_BINARY_DIR}/toc.bin)

add_custom_command(OUTPUT ${TOC_BINARY_OUTPUT}
	COMMAND ${CMAKE_OBJCOPY} -O binary ${TOC_NAME} ${TOC_BINARY_OUTPUT}
	DEPENDS toc
)
add_custom_target(toc_bin DEPENDS ${TOC_BINARY_OUTPUT})
