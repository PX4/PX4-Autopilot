px4_add_module(
	MODULE modules__formic_watchdog_ev
	MAIN formic_watchdog_ev
	SRCS
		formic_watchdog_ev.cpp
	DEPENDS
		px4_work_queue
) 

