include_directories(${CMAKE_CURRENT_BINARY_DIR})


px4_posix_generate_builtin_commands(
	OUT apps.h
	MODULE_LIST ${module_libraries})

px4_generate_parameters()

add_executable(mainapp
	${CMAKE_SOURCE_DIR}/src/platforms/posix/main.cpp
	${CMAKE_BINARY_DIR}/parameters.c
	apps.h
	)

target_link_libraries(mainapp
	-Wl,--start-group
	${module_libraries}
	pthread m rt
	-Wl,--end-group
	)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
