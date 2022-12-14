############################################################################
#
# Copyright (c) 2022 ModalAI, Inc. All rights reserved.
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

set(HEXAGON_SDK_INCLUDES
	${HEXAGON_SDK_ROOT}/incs
	${HEXAGON_SDK_ROOT}/incs/stddef
	${HEXAGON_SDK_ROOT}/rtos/qurt/computev66/include/qurt
	${HEXAGON_SDK_ROOT}/rtos/qurt/computev66/include/posix
	${HEXAGON_SDK_ROOT}/tools/HEXAGON_Tools/8.4.05/Tools/target/hexagon/include
	)

set(TOOLSLIB ${HEXAGON_TOOLS_ROOT}/target/hexagon/lib/${V_ARCH}/G0/pic)

set(HEXAGON_ARCH_FLAGS
	-march=hexagon
	-mcpu=hexagon${V_ARCH}
	)

macro (list2string out in)
	set(list ${ARGV})
	list(REMOVE_ITEM list ${out})
	foreach(item ${list})
		set(${out} "${${out}} ${item}")
	endforeach()
endmacro(list2string)

set(HEXAGON_START_LINK_FLAGS)
list2string(HEXAGON_START_LINK_FLAGS
	${HEXAGON_ARCH_FLAGS}
	-shared
	-call_shared
	-G0
	${TOOLSLIB}/initS.o
	"-o <TARGET>"
	-L${TOOLSLIB}
	-Bsymbolic
	${TOOLSLIB}/libgcc.a
	--wrap=malloc
	--wrap=calloc
	--wrap=free
	--wrap=realloc
	--wrap=memalign
	--wrap=__stack_chk_fail
	-lc
	"-soname=<TARGET_SONAME>"
	)

set(HEXAGON_END_LINK_FLAGS)
list2string(HEXAGON_END_LINK_FLAGS
	--start-group
	-lgcc
	--end-group
	${TOOLSLIB}/finiS.o
	)

set(CMAKE_C_CREATE_SHARED_LIBRARY
	"${HEXAGON_LINK} ${HEXAGON_START_LINK_FLAGS} --start-group --whole-archive <OBJECTS> <LINK_LIBRARIES> --end-group ${HEXAGON_END_LINK_FLAGS}")

set(CMAKE_CXX_CREATE_SHARED_LIBRARY
	"${HEXAGON_LINK} ${HEXAGON_START_LINK_FLAGS} --start-group ${TOOLSLIB}/libc++.so.1 ${TOOLSLIB}/libc++abi.so.1 ${HEXAGON_SDK_ROOT}/libs/weak_refs/ship/hexagon_toolv84/weak_refs.so --end-group --start-group --whole-archive <OBJECTS> <LINK_LIBRARIES> --end-group ${HEXAGON_END_LINK_FLAGS}")

set(DYNAMIC_LIBS -Wl,${TOOLSLIB}/libc++.a)

# Base CPU flags for each of the supported architectures.
set(ARCHCPUFLAGS
	-m${V_ARCH}
	-G0
	)

add_definitions(
	-D __QURT
	# -D DEBUG_BUILD # Add this to get extra debug output
	-D _PROVIDE_POSIX_TIME_DECLS
	-D _HAS_C9X
	-D restrict=__restrict__
	-D noreturn_function=
	)

# Language-specific flags
set(ARCHCFLAGS
	-D__CUSTOM_FILE_IO__
	)

set(ARCHCXXFLAGS
	-DCONFIG_WCHAR_BUILTIN
	-D__CUSTOM_FILE_IO__
	)

exec_program(${CMAKE_CXX_COMPILER} ${CMAKE_CURRENT_SOURCE_DIR} ARGS -print-libgcc-file-name OUTPUT_VARIABLE LIBGCC)
exec_program(${CMAKE_CXX_COMPILER} ${CMAKE_CURRENT_SOURCE_DIR} ARGS -print-file-name=libm.a OUTPUT_VARIABLE LIBM)
set(EXTRA_LIBS ${EXTRA_LIBS} ${LIBM})

# Full optimization and Link Time Optimization (LTO)
set(QURT_OPTIMIZATION_FLAGS -O3 -flto)

# Flags we pass to the C compiler
list2string(CFLAGS
	${ARCHCFLAGS}
	${ARCHCPUFLAGS}
	${QURT_OPTIMIZATION_FLAGS}
	${HEXAGON_INCLUDE_DIRS}
	)

# Flags we pass to the C++ compiler
list2string(CXXFLAGS
    -Wno-inconsistent-missing-override
	${ARCHCXXFLAGS}
	${ARCHCPUFLAGS}
	${QURT_OPTIMIZATION_FLAGS}
	${HEXAGON_INCLUDE_DIRS}
	)

# Set cmake flags
list2string(QURT_CMAKE_C_FLAGS
	${CMAKE_C_FLAGS}
	${CFLAGS}
	)

set(CMAKE_C_FLAGS "${QURT_CMAKE_C_FLAGS}")

list2string(QURT_CMAKE_CXX_FLAGS
	${CMAKE_CXX_FLAGS}
	${CXXFLAGS}
	)

set(CMAKE_CXX_FLAGS ${QURT_CMAKE_CXX_FLAGS})

# Flags we pass to the linker
# CMake make test builds of apps to validate the compiler
# These settings enable CMake to build the required test apps
list2string(CMAKE_EXE_LINKER_FLAGS
	-m${V_ARCH}
	-mG0lib
	-G0
	-fpic
	-shared
	-Wl,-Bsymbolic
	-Wl,--wrap=malloc
	-Wl,--wrap=calloc
	-Wl,--wrap=free
	-Wl,--wrap=realloc
	-Wl,--wrap=memalign
	-Wl,--wrap=__stack_chk_fail
	${DYNAMIC_LIBS}
	-lc
	${EXTRALDFLAGS}
	)

include (CMakeParseArguments)
