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

add_custom_target(run_sitl_quad
	COMMAND Tools/sitl_run.sh posix-configs/SITL/init/rcS "${DEBUGGER}" jmavsim "${CMAKE_BINARY_DIR}"
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	)
add_dependencies(run_sitl_quad mainapp)

add_custom_target(run_sitl_iris
	COMMAND Tools/sitl_run.sh posix-configs/SITL/init/rcS_iris_gazebo ${DEBUGGER} jmavsim ${CMAKE_BINARY_DIR}
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	)
add_dependencies(run_sitl_iris mainapp)

add_custom_target(run_sitl_plane
	COMMAND Tools/sitl_run.sh posix-configs/SITL/init/rc.fixed_wing ${DEBUGGER} none ${CMAKE_BINARY_DIR}
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	)
add_dependencies(run_sitl_plane mainapp)

add_custom_target(run_sitl_ros
	COMMAND Tools/sitl_run.sh posix-configs/SITL/init/rc_iris_ros none  ${DEBUGGER} none ${CMAKE_BINARY_DIR}
	WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
	)
add_dependencies(run_sitl_ros mainapp)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
