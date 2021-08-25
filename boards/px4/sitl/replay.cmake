
px4_add_board(
	PLATFORM posix
	ROMFSROOT px4fmu_common
	MODULES
		ekf2
		logger
		replay
	SYSTEMCMDS
		param
		perf
		shutdown
		topic_listener
		ver
		work_queue
	)

message(STATUS "Building with uorb publisher rules support")
add_definitions(-DORB_USE_PUBLISHER_RULES)

set(ENABLE_LOCKSTEP_SCHEDULER yes)
