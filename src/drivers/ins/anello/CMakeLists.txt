# SPDX-License-Identifier: BSD-3-Clause
px4_add_module(
	MODULE drivers__ins__anello
	MAIN anello
	SRCS
		Anello.cpp
		AnelloParser.hpp
	MODULE_CONFIG
		module.yaml
	DEPENDS
		geo
		px4_work_queue
)

# MAVLink is configured after drivers; defer interface resolution to generation.
target_include_directories(drivers__ins__anello PRIVATE
	$<TARGET_PROPERTY:mavlink_c,INTERFACE_INCLUDE_DIRECTORIES>)
target_compile_options(drivers__ins__anello PRIVATE
	$<TARGET_PROPERTY:mavlink_c,INTERFACE_COMPILE_OPTIONS>)
add_dependencies(drivers__ins__anello mavlink_c_generate)
