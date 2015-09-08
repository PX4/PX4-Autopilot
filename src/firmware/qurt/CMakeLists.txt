set(module_list)
px4_qurt_add_modules(module_list ${BOARD})
message(STATUS "module list: ${module_list}")

px4_qurt_generate_builtin_commands(
	OUT builtin_commands.c
	MODULE_LIST ${module_list})

add_executable(mainapp builtin_commands.c)

# vim: set noet ft=cmake fenc=utf-8 ff=unix :
