set(V_ARCH v5)
set(HEXAGON_TOOLS_ROOT /opt/6.4.03)
set(TOOLSLIB 
	${HEXAGON_TOOLS_ROOT}/dinkumware/lib/${V_ARCH}/G0)
set(module_dir_list)
set(module_list)
px4_qurt_add_modules(module_dir_list ${BOARD})
message(STATUS "module list: ${module_dir_list}")

foreach(directory ${module_dir_list})
	message(STATUS "directory: ${directory}")
	px4_mangle_name(${directory} mangled_name)
	list(APPEND module_list
		${mangled_name})
endforeach()
px4_qurt_generate_builtin_commands(
	OUT builtin_commands.cpp
	MODULE_LIST ${module_list})

# FIXME @jgoppert - how to work around issues like this?
# Without changing global variables?
# Clear -rdynamic flag which fails for hexagon
set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")

add_executable(mainapp builtin_commands.cpp)
target_link_libraries(mainapp
	-Wl,--whole-archive
	${module_list}
	m
	-Wl,--no-whole-archive
	-Wl,${TOOLSLIB}/pic/libstdc++.a)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
