px4_add_git_submodule(TARGET git_jmavsim PATH "${PX4_SOURCE_DIR}/Tools/simulation/jmavsim/jMAVSim")

# create targets for each viewer/model/debugger combination
set(debuggers
	none
	gdb
	lldb
	valgrind
	callgrind
)

foreach(debugger ${debuggers})
	if(debugger STREQUAL "none")
		set(_targ_name "jmavsim")
	else()
		set(_targ_name "jmavsim_${debugger}")
	endif()

	add_custom_target(${_targ_name}
		COMMAND ${PX4_SOURCE_DIR}/Tools/simulation/jmavsim/sitl_run.sh $<TARGET_FILE:px4> ${debugger} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS px4 git_jmavsim
	)
endforeach()
