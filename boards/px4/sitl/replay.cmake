
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

set(ENABLE_LOCKSTEP_SCHEDULER yes)
