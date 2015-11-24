include_directories(${CMAKE_CURRENT_BINARY_DIR})


px4_posix_generate_builtin_commands(
	OUT apps.h
	MODULE_LIST ${module_libraries})

add_executable(mainapp
	${CMAKE_SOURCE_DIR}/src/platforms/posix/main.cpp
	apps.h
	)

if (NOT ${CMAKE_C_COMPILER_ID} STREQUAL "Clang" OR NOT APPLE)
	target_link_libraries(mainapp
		-Wl,--start-group
		${module_libraries}
		pthread m rt
		-Wl,--end-group
		)
else()
	target_link_libraries(mainapp
		${module_libraries}
		pthread m
		)
endif()

add_custom_target(run_config
	COMMAND Tools/sitl_run.sh "${config_sitl_rcS}" "${config_sitl_debugger}"
	"${config_sitl_viewer}" "${config_sitl_model}" "${CMAKE_BINARY_DIR}"
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	USES_TERMINAL
	)
add_dependencies(run_config mainapp)

foreach(viewer none jmavsim gazebo)
	foreach(debugger none gdb lldb ddd valgrind)
		foreach(model none iris tailsitter)
			if (debugger STREQUAL "none")
				if (model STREQUAL "none")
					set(_targ_name "${viewer}")
				else()
					set(_targ_name "${viewer}_${model}")
				endif()
			else()
				if (model STREQUAL "none")
					set(_targ_name "${viewer}___${debugger}")
				else()
					set(_targ_name "${viewer}_${model}_${debugger}")
				endif()
			endif()
			add_custom_target(${_targ_name}
				COMMAND Tools/sitl_run.sh "${config_sitl_rcS}"
				"${debugger}"
				"${viewer}" "${model}" "${CMAKE_BINARY_DIR}"
				WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
				USES_TERMINAL
				)
			add_dependencies(${_targ_name} mainapp)
		endforeach()
	endforeach()
endforeach()

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
