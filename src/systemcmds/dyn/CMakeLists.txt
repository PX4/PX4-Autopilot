px4_add_module(
	MODULE systemcmds__dyn
	MAIN dyn
	SRCS
		dyn.cpp
	)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Windows")
	# Windows has no libdl — dlopen/dlsym are provided as inlines over
	# LoadLibrary/GetProcAddress in the POSIX shim, so no extra link
	# dependency is needed on that platform.
	target_link_libraries(systemcmds__dyn PUBLIC dl)
endif()
