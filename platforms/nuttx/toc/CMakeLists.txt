############################################################################
#
#   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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

# ---------------------------------------------------------------------------
# Standalone TOC block for boards whose bootloader expects a separate signed
# TOC prepended to the signed app payload on media.
#
# The board opts in by shipping BOTH:
#   - <board>/src/toc.c                         (const image_toc_entry_t table)
#   - <board>/nuttx-config/scripts/toc.ld       (standalone TOC linker script)
#
# When present, this subdir:
#   1. builds board_toc (from the board's toc.c) *without* linking it into
#      drivers_board / the app ELF — TOC only exists as the standalone block,
#      not embedded in the app image;
#   2. links toc.elf against toc.ld with fw_image.c (.incbin of the unsigned
#      app .bin) so toc.ld can compute _app_start/_app_end from the real
#      payload size;
#   3. objcopies toc.elf -> toc.bin;
#   4. signs toc.bin with sign_firmware.py (same key as CONFIG_BOARD_SECUREBOOT
#      uses for the app);
#   5. exposes the signed TOC via PX4_SIGNED_TOC_BIN so the shared .px4
#      wrapper in platforms/nuttx/CMakeLists.txt prepends it to the signed
#      app before packing the .px4.
#
# This is not used by the boards that embed the TOC inside the app ELF via
# their own script.ld
# ---------------------------------------------------------------------------

set(_board_toc_ld ${PX4_BOARD_DIR}/nuttx-config/scripts/toc.ld)
set(_board_toc_c  ${PX4_BOARD_DIR}/src/toc.c)

if(NOT (EXISTS ${_board_toc_ld} AND EXISTS ${_board_toc_c}))
	return()
endif()

if(NOT CONFIG_BOARD_SECUREBOOT)
	message(WARNING
		"Board provides toc.ld/toc.c but CONFIG_BOARD_SECUREBOOT is off; "
		"standalone TOC will not be built/signed.")
	return()
endif()

# 1. Board TOC table. Not linked into drivers_board — TOC is media-only.
add_library(board_toc ${_board_toc_c})
add_dependencies(board_toc nuttx_context)

# 2. toc.elf: fw_image.c .incbin's the unsigned app .bin.
set(_app_bin ${PX4_BINARY_DIR}/${PX4_CONFIG}.bin)

add_executable(toc_elf ${CMAKE_CURRENT_LIST_DIR}/fw_image.c)
set_source_files_properties(${CMAKE_CURRENT_LIST_DIR}/fw_image.c PROPERTIES
	OBJECT_DEPENDS ${_app_bin})
target_compile_options(toc_elf PRIVATE
	-DPX4_UNSIGNED_FIRMWARE=${_app_bin})
target_link_options(toc_elf PRIVATE
	-nostartfiles -nodefaultlibs -nostdlib
	-Wl,--script=${_board_toc_ld}
	-Wl,--warn-common
	-Wl,-Map=${PX4_BINARY_DIR}/toc.map
)
target_link_libraries(toc_elf PRIVATE board_toc)
set_target_properties(toc_elf PROPERTIES
	OUTPUT_NAME ${PX4_BINARY_DIR}/toc.elf)
add_dependencies(toc_elf px4)

# 3. objcopy toc.elf -> toc.bin.
set(_toc_bin ${PX4_BINARY_DIR}/toc.bin)
add_custom_command(OUTPUT ${_toc_bin}
	COMMAND ${CMAKE_OBJCOPY} -O binary
		${PX4_BINARY_DIR}/toc.elf ${_toc_bin}
	DEPENDS toc_elf
)

# 4. Sign toc.bin. Key selection matches the app-signing block in
#    platforms/nuttx/CMakeLists.txt (env var overrides Kconfig).
if(DEFINED ENV{BOARD_SECUREBOOT_KEY})
	set(_toc_key $ENV{BOARD_SECUREBOOT_KEY})
else()
	set(_toc_key ${CONFIG_BOARD_SECUREBOOT_KEY})
endif()
if(NOT IS_ABSOLUTE ${_toc_key})
	get_filename_component(_toc_key ${_toc_key} ABSOLUTE
		BASE_DIR ${PX4_SOURCE_DIR})
endif()

set(_toc_signed ${PX4_BINARY_DIR}/toc_signed.bin)
add_custom_command(OUTPUT ${_toc_signed}
	COMMAND ${PYTHON_EXECUTABLE}
		${PX4_SOURCE_DIR}/Tools/secure_bootloader/sign_firmware.py
		--key ${_toc_key} ${_toc_bin} ${_toc_signed}
	DEPENDS ${_toc_bin}
	COMMENT "Signing standalone toc.bin"
)

add_custom_target(toc_signed ALL DEPENDS ${_toc_signed})

# 5. Publish to the shared .px4 wrapper.
set(PX4_SIGNED_TOC_BIN ${_toc_signed}
	CACHE INTERNAL "Signed standalone TOC prepended to signed app")
