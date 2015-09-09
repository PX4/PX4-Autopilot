include(px4_base)

set(module_dir_list)
set(module_list)
px4_posix_add_modules(module_dir_list ${BOARD})
message(STATUS "module list: ${module_dir_list}")

include_directories(${CMAKE_CURRENT_BINARY_DIR})

foreach(directory ${module_dir_list})
	message(STATUS "directory: ${directory}")
	px4_mangle_name(${directory} mangled_name)
	list(APPEND module_list
		${mangled_name})
endforeach()
px4_posix_generate_builtin_commands(
	OUT apps.h
	MODULE_LIST ${module_list})

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
	${module_list}
	pthread m
	-Wl,--end-group
	)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
