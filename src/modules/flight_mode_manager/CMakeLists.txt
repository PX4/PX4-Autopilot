############################################################################
#
#   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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


###########################################
# Prepare flight tasks
###########################################

# add core flight tasks to list
set(flight_tasks_all)
list(APPEND flight_tasks_all
	Auto
	Descend
	Failsafe
	ManualAcceleration
	ManualAltitude
	ManualAltitudeSmoothVel
	ManualPosition
	ManualPositionSmoothVel
	Transition
)

if(NOT px4_constrained_flash_build)
	list(APPEND flight_tasks_all
		AutoFollowTarget
		Orbit
	)
endif()

# set the files to be generated
set(files_to_generate
	FlightTasks_generated.hpp
	FlightTasks_generated.cpp
)

# generate files needed for Flight Tasks
set(python_args
	-t ${flight_tasks_all}
	-i ${CMAKE_CURRENT_SOURCE_DIR}/Templates
	-o ${CMAKE_CURRENT_BINARY_DIR}
	-f ${files_to_generate}
)

# generate the files using the python script and template
add_custom_command(
	OUTPUT
		${CMAKE_CURRENT_BINARY_DIR}/FlightTasks_generated.hpp
		${CMAKE_CURRENT_BINARY_DIR}/FlightTasks_generated.cpp
	COMMAND ${PYTHON_EXECUTABLE} generate_flight_tasks.py ${python_args}
	COMMENT "Generating Flight Tasks"
	DEPENDS
		Templates/FlightTasks_generated.cpp.em
		Templates/FlightTasks_generated.hpp.em
	WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
	VERBATIM
)
add_custom_target(flighttasks_generated
	DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/FlightTasks_generated.cpp)
add_dependencies(prebuild_targets flighttasks_generated)

add_compile_options(
	-Wno-cast-align
) # TODO: fix and enable

# add subdirectory containing all tasks
add_subdirectory(tasks)

px4_add_module(
	MODULE modules__flight_mode_manager
	MAIN flight_mode_manager
	COMPILE_FLAGS
	INCLUDES
		${CMAKE_CURRENT_SOURCE_DIR}
		${CMAKE_CURRENT_BINARY_DIR}
	SRCS
		FlightModeManager.cpp
		FlightModeManager.hpp

		${CMAKE_CURRENT_BINARY_DIR}/FlightTasks_generated.hpp
		${CMAKE_CURRENT_BINARY_DIR}/FlightTasks_generated.cpp
	DEPENDS
		px4_work_queue
		WeatherVane
		flighttasks_generated
)

# add all flight task dependencies
foreach(task ${flight_tasks_all})
	target_link_libraries(modules__flight_mode_manager PUBLIC FlightTask${task})
endforeach()
