include(px4_base)

set(module_dir_list)
set(module_list)
px4_posix_add_modules(module_dir_list ${BOARD})
message(STATUS "module list: ${module_dir_list}")

foreach(directory ${module_dir_list})
	message(STATUS "directory: ${directory}")
	px4_mangle_name(${directory} mangled_name)
	list(APPEND module_list
		${mangled_name})
endforeach()
px4_posix_generate_builtin_commands(
	OUT builtin_commands.cpp
	MODULE_LIST ${module_list})

add_executable(mainapp builtin_commands.cpp)
target_link_libraries(mainapp
	-Wl,--start-group
	${module_list}
	pthread m
	-Wl,--end-group
	)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
