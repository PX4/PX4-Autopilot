############################################################################
#
# Catapult Launch Automation Module (PoC - Approach B)
#
############################################################################

px4_add_module(
	MODULE modules__catapult_launch
	MAIN catapult_launch
	COMPILE_FLAGS
		-Wno-unused-variable
	SRCS
		CatapultLaunch.cpp
	DEPENDS
		px4_work_queue
)
