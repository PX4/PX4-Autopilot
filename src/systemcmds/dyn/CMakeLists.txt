px4_add_module(
	MODULE systemcmds__dyn
	MAIN dyn
	SRCS
		dyn.cpp
	)

target_link_libraries(systemcmds__dyn PUBLIC dl)
