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

#=============================================================================
#
#	make_bin_romfs
#
#	This function creates a ROMFS from the input binary directory and places
#	the generated C-style header into the specified output directory. To
#	reduce the ROMFS size, an option is given to strip the input files, which
#	reduces their size considerably
#
#	Usage:
#		make_bin_romfs(
#			BINDIR <string>
#			OUTDIR <string>
#			OUTPREFIX <string>
#			LABEL <string>
#			STRIPPED <string>
#			DEPS <list>
#		)
#
#	Input:
#		BINDIR		: Input binary directory (source for ROMFS)
#		OUTDIR  	: Output directory (${OUTPREFIX}_romfs.h is placed here)
#		OUTPREFIX	: Prefix for the output file
#		LABEL		: Volume label
#		STRIPPED	: Set to 'y' to strip the input binaries
#		DEPS		: Dependencies for the ROMFS
#
#	Output:
#		${OUTPREFIX}_romfs.h file, that can be added to any C-source file.
#		Mounting as a ROM-disk will be trivial once this is done.
#
function(make_bin_romfs)

	px4_parse_function_args(
		NAME make_bin_romfs
		ONE_VALUE BINDIR OUTDIR OUTPREFIX LABEL STRIPPED
		MULTI_VALUE DEPS
		REQUIRED BINDIR OUTDIR OUTPREFIX DEPS
		ARGN ${ARGN}
	)

	if (NOT STRIPPED)
		set(STRIPPED "n")
	endif()

	if (NOT LABEL)
		set(LABEL "NuttXRomfsVol")
	endif()

	# Strip the elf files to reduce romfs image size

	if (${STRIPPED} STREQUAL "y")
		# Preserve the files with debug symbols

		add_custom_target(debug_${OUTPREFIX}
			COMMAND cp -r ${BINDIR} ${BINDIR}_debug
			DEPENDS ${DEPS}
		)

		# Then strip the binaries in place

		add_custom_command(OUTPUT ${OUTPREFIX}_stripped_bins
			COMMAND for f in * \; do if [ -f "$$f" ]; then ${CMAKE_STRIP} $$f --strip-unneeded \; fi \; done
			DEPENDS ${DEPS} debug_${OUTPREFIX}
			WORKING_DIRECTORY ${BINDIR}
		)
	else()
		add_custom_command(OUTPUT ${OUTPREFIX}_stripped_bins
			COMMAND touch ${BINDIR}
		)
	endif()

	# Make sure we have what we need

	find_program(GENROMFS genromfs)
	if(NOT GENROMFS)
		message(FATAL_ERROR "genromfs not found")
	endif()

	find_program(XXD xxd)
	if(NOT XXD)
		message(FATAL_ERROR "xxd not found")
	endif()

	find_program(SED sed)
	if(NOT SED)
		message(FATAL_ERROR "sed not found")
	endif()

	# Generate the ROM file system

	add_custom_command(OUTPUT ${OUTDIR}/${OUTPREFIX}_romfsimg.h
		COMMAND ${GENROMFS} -f ${OUTPREFIX}_romfs.img -d ${BINDIR} -V "${LABEL}"
		COMMAND ${XXD} -i ${OUTPREFIX}_romfs.img |
				${SED} 's/^unsigned char/const unsigned char/g' >${OUTPREFIX}_romfsimg.h
		COMMAND mv ${OUTPREFIX}_romfsimg.h ${OUTDIR}/${OUTPREFIX}_romfsimg.h
		COMMAND rm -f ${OUTPREFIX}_romfs.img
		DEPENDS ${OUTDIR} ${DEPS} ${OUTPREFIX}_stripped_bins
	)
	add_custom_target(${OUTPREFIX}_romfsimg DEPENDS ${OUTDIR}/${OUTPREFIX}_romfsimg.h)

endfunction()
