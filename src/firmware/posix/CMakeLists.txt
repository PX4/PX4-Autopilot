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

add_custom_target(run_sitl
	COMMAND Tools/sitl_run.sh "${config_sitl_rcS}" "${config_sitl_debugger}"
	"${config_sitl_viewer}" "${CMAKE_BINARY_DIR}"
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	)
add_dependencies(run_sitl mainapp)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
