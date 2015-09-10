include_directories(${CMAKE_CURRENT_BINARY_DIR})

px4_posix_generate_builtin_commands(
	OUT apps.h
	MODULE_LIST ${module_libraries})

add_executable(mainapp
	${CMAKE_SOURCE_DIR}/src/platforms/posix/main.cpp
	apps.h
	)

set(main_link_flags
	"-T${CMAKE_SOURCE_DIR}/cmake/posix/ld.script"
	)
px4_join(OUT main_link_flags LIST ${main_link_flags} GLUE " ")
set_target_properties(mainapp PROPERTIES LINK_FLAGS ${main_link_flags})

target_link_libraries(mainapp
	-Wl,--start-group
	${module_libraries}
	pthread m
	-Wl,--end-group
	)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
