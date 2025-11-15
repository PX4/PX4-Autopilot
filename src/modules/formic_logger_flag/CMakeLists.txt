px4_add_module(
	MODULE modules__formic_logger_flag
	MAIN formic_logger_flag
	SRCS
		formic_logger_flag.cpp
	DEPENDS
		px4_work_queue
) 

