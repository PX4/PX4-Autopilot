px4_add_module(
	MODULE modules__interceptor_control
	MAIN interceptor_control
	SRCS
		interceptor_control.cpp
		interceptor_control.hpp
		pitch_damper.cpp
		roll_damper.cpp
		yaw_damper.cpp
	DEPENDS
		uORB
		px4_work_queue
	)
