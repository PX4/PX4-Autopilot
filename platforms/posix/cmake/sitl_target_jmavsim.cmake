
px4_add_git_submodule(TARGET git_jmavsim PATH "${PX4_SOURCE_DIR}/Tools/jMAVSim")

set(debuggers
	none
	gdb
	lldb
	valgrind
	callgrind
)

# jmavsim iris
set(model iris)
set(viewer jmavsim)
foreach(debugger ${debuggers})

	if(debugger STREQUAL "none")
		if(model STREQUAL "none")
			set(_targ_name "${viewer}")
		else()
			set(_targ_name "${viewer}_${model}")
		endif()
	else()
		if(model STREQUAL "none")
			set(_targ_name "${viewer}___${debugger}")
		else()
			set(_targ_name "${viewer}_${model}_${debugger}")
		endif()
	endif()

	add_custom_target(${_targ_name}
		COMMAND
			${PX4_SOURCE_DIR}/Tools/sitl_run.sh $<TARGET_FILE:px4> ${debugger} ${viewer} ${model} ${world} ${PX4_SOURCE_DIR} ${PX4_BINARY_DIR}
		WORKING_DIRECTORY ${SITL_WORKING_DIR}
		USES_TERMINAL
		DEPENDS
			git_jmavsim
			logs_symlink
			px4
	)

endforeach()
