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

# kill all flags above us, this is a different board (io)
set_directory_properties(PROPERTIES
	INCLUDE_DIRECTORIES ""
	LINK_DIRECTORIES ""
	COMPILE_DEFINITIONS ""
	)
set(c_flags)
set(exe_linker_flags)
set(cxx_flags)
set(include_dirs)
set(link_dirs)
set(definitions)

px4_os_prebuild_targets(OUT io_prebuild_targets
	BOARD ${config_io_board}
	THREADS ${THREADS})

px4_os_add_flags(
	BOARD ${config_io_board}
	C_FLAGS c_flags
	CXX_FLAGS cxx_flags
	EXE_LINKER_FLAGS exe_linker_flags
	INCLUDE_DIRS include_dirs
	LINK_DIRS link_dirs
	DEFINITIONS definitions)

px4_join(OUT CMAKE_EXE_LINKER_FLAGS LIST "${exe_linker_flags}" GLUE " ")
px4_join(OUT CMAKE_C_FLAGS LIST "${c_flags}" GLUE " ")
px4_join(OUT CMAKE_CXX_FLAGS LIST "${cxx_flags}" GLUE " ")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${EXTRA_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${EXTRA_CXX_FLAGS}")

include_directories(
	${include_dirs}
	${CMAKE_BINARY_DIR}/src/modules/systemlib/mixer
	.
	)
link_directories(${link_dirs})
add_definitions(${definitions})

set(srcs
	adc.c
	controls.c
	px4io.c
	registers.c
	safety.c
	../systemlib/up_cxxinitialize.c
	../systemlib/perf_counter.c
	mixer.cpp
	../systemlib/mixer/mixer.cpp
	../systemlib/mixer/mixer_group.cpp
	../systemlib/mixer/mixer_multirotor.cpp
	../systemlib/mixer/mixer_simple.cpp
	../systemlib/pwm_limit/pwm_limit.c
	../../lib/rc/st24.c
	../../lib/rc/sumd.c
	../../lib/rc/sbus.c
	../../lib/rc/dsm.c
	../../lib/rc/srxl.c
	../../drivers/stm32/drv_hrt.c
	../../drivers/stm32/drv_io_timer.c
	../../drivers/stm32/drv_pwm_servo.c
	../../drivers/boards/${config_io_board}/px4io_init.c
	../../drivers/boards/${config_io_board}/px4io_timer_config.c
	)

if(${config_io_board} STREQUAL "px4io-v1")
	list(APPEND srcs
		i2c.c
		)
elseif(${config_io_board} STREQUAL "px4io-v2")
	list(APPEND srcs
		serial.c
		../systemlib/hx_stream.c
		)
endif()

set(fw_io_name ${config_io_board})

add_executable(${fw_io_name}
	${srcs})

add_dependencies(${fw_io_name}
	nuttx_export_${config_io_board}
	msg_gen
	io_prebuild_targets
	mixer_gen
	)

set(nuttx_export_dir ${CMAKE_BINARY_DIR}/${config_io_board}/NuttX/nuttx-export)
set(main_link_flags
	"-T${nuttx_export_dir}/build/ld.script"
	"-Wl,-Map=${CMAKE_BINARY_DIR}/${config_io_board}/main.map"
	)
px4_join(OUT main_link_flags LIST ${main_link_flags} GLUE " ")
set_target_properties(${fw_io_name} PROPERTIES LINK_FLAGS ${main_link_flags})

target_link_libraries(${fw_io_name}
	-Wl,--start-group
	apps nuttx nosys m gcc
	${config_io_extra_libs}
	-Wl,--end-group)

px4_nuttx_create_bin(OUT ${CMAKE_CURRENT_BINARY_DIR}/${fw_io_name}.bin
	EXE ${fw_io_name}
	)

add_custom_target(fw_io
	DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${fw_io_name}.bin)


# vim: set noet ft=cmake fenc=utf-8 ff=unix : 
