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

# Build the CM4 rpmsg remote with the firmware toolchain and register it as a
# ROMFS extra (/etc/extras/cm4_rpmsg.elf), where the rptun loader reads it.

set(CM4_ELF ${CMAKE_CURRENT_BINARY_DIR}/cm4_rpmsg.elf)

# CONFIG_BOARD_CM4_APP (root Kconfig, BOARD_RPMSG) picks apps/<name>/
if(NOT CM4_APP)
	set(CM4_APP basic)
endif()

file(GLOB CM4_SOURCES CONFIGURE_DEPENDS
	${CMAKE_CURRENT_SOURCE_DIR}/*.c ${CMAKE_CURRENT_SOURCE_DIR}/*.h
	${CMAKE_CURRENT_SOURCE_DIR}/apps/${CM4_APP}/*.c ${CMAKE_CURRENT_SOURCE_DIR}/apps/${CM4_APP}/*.h)

add_custom_command(OUTPUT ${CM4_ELF}
	COMMAND make --no-print-directory -C ${CMAKE_CURRENT_SOURCE_DIR}
		CC=${CMAKE_C_COMPILER} STRIP=${CMAKE_STRIP}
		BUILDDIR=${CMAKE_CURRENT_BINARY_DIR} APP=${CM4_APP}
		PX4_BUILD=${PX4_BINARY_DIR} PX4_SRC=${PX4_SOURCE_DIR}
	DEPENDS ${CM4_SOURCES} ${CMAKE_CURRENT_SOURCE_DIR}/link.ld.in ${CMAKE_CURRENT_SOURCE_DIR}/Makefile
	COMMENT "Building CM4 rpmsg remote, application ${CM4_APP}"
)

add_custom_target(cm4_firmware DEPENDS ${CM4_ELF})

# consumed by ROMFS/CMakeLists.txt, which is processed after the board sources
set(config_romfs_extra_files ${config_romfs_extra_files} ${CM4_ELF}
	CACHE INTERNAL "extra ROMFS files" FORCE)
set(config_romfs_extra_dependencies ${config_romfs_extra_dependencies} cm4_firmware
	CACHE INTERNAL "extra ROMFS deps" FORCE)
