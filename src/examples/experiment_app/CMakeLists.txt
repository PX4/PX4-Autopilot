## experiments CMakeLists.

px4_add_module(
	MODULE examples__experiment_app
	MAIN experiment
	STACK_MAIN 2000
	SRCS
		experiment.cpp
		experiment.h
	DEPENDS
	)
