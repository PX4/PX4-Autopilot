############################################################################
#
# Copyright (c) 2019 PX4 Development Team. All rights reserved.
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

find_program(BLOATY_PROGRAM bloaty)
if(BLOATY_PROGRAM)

	set(BLOATY_OPTS --demangle=full --domain=vm -s vm -n 200 -w)

	# bloaty compilation units
	add_custom_target(bloaty_compileunits
		COMMAND ${BLOATY_PROGRAM} -d compileunits ${BLOATY_OPTS} $<TARGET_FILE:px4>
		DEPENDS px4
		USES_TERMINAL
		)

	# bloaty inlines
	add_custom_target(bloaty_inlines
		COMMAND ${BLOATY_PROGRAM} -d inlines ${BLOATY_OPTS} $<TARGET_FILE:px4>
		DEPENDS px4
		USES_TERMINAL
		)

	# bloaty segments,sections
	add_custom_target(bloaty_segments
		COMMAND ${BLOATY_PROGRAM} -d segments,sections ${BLOATY_OPTS} $<TARGET_FILE:px4>
		DEPENDS px4
		USES_TERMINAL
		)

	# bloaty symbols
	add_custom_target(bloaty_symbols
		COMMAND ${BLOATY_PROGRAM} -d symbols ${BLOATY_OPTS} $<TARGET_FILE:px4>
		DEPENDS px4
		USES_TERMINAL
		)

	# bloaty templates
	add_custom_target(bloaty_templates
		COMMAND ${BLOATY_PROGRAM} -d shortsymbols,fullsymbols ${BLOATY_OPTS} $<TARGET_FILE:px4>
		DEPENDS px4
		USES_TERMINAL
		)

	# bloaty statically allocated RAM
	add_custom_target(bloaty_ram
		COMMAND ${BLOATY_PROGRAM} -c ${PX4_SOURCE_DIR}/Tools/bloaty_static_ram.bloaty -d bloaty_static_ram,compileunits --source-filter ^ram$ ${BLOATY_OPTS} $<TARGET_FILE:px4>
		DEPENDS px4
		USES_TERMINAL
		)

	if(${PX4_PLATFORM} MATCHES "nuttx")
		# bloaty compare with last master build
		add_custom_target(bloaty_compare_master
			COMMAND wget --continue --no-verbose https://s3.amazonaws.com/px4-travis/Firmware/master/${PX4_BOARD_VENDOR}_${PX4_BOARD_MODEL}_${PX4_BOARD_LABEL}.elf -O master.elf
			COMMAND ${BLOATY_PROGRAM} -d symbols ${BLOATY_OPTS} $<TARGET_FILE:px4> -- master.elf
			DEPENDS px4
			WORKING_DIRECTORY ${PX4_BINARY_DIR}
			VERBATIM
			USES_TERMINAL
			)
	endif()
endif()
