px4_add_module(
	MODULE stm32_motor_ctrl
	MAIN stm32_motor_ctrl
	SRCS
		Stm32MotorCtrl.cpp
	DEPENDS
		px4_work_queue
)


