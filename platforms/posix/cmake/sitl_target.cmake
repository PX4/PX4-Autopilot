
set(debuggers
	none
	gdb
	lldb
	valgrind
	callgrind
)

set(model none)
set(viewer none)
set(world none)

foreach(debugger ${debuggers})

	if(debugger STREQUAL "none")
		set(_targ_name "${viewer}")
	else()
		set(_targ_name "${viewer}___${debugger}")
	endif()

	add_custom_target(${_targ_name}
		#COMMAND mkdir -p ${PX4_BINARY_DIR}/tmp/rootfs
		COMMAND
			$<TARGET_FILE:px4> ${PX4_BINARY_DIR}/etc -s etc/init.d-posix/rcS -t ${PX4_SOURCE_DIR}/test_data
		DEPENDS logs_symlink px4
		WORKING_DIRECTORY ${PX4_BINARY_DIR}
		USES_TERMINAL
	)

endforeach()
