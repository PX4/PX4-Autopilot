px4_add_module(
	MODULE stm32_bridge
	MAIN stm32_bridge
	SRCS
		Stm32Bridge.cpp
	DEPENDS
		px4_work_queue
)


