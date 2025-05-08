px4_add_module(
	MODULE modules__robosub__rs_pwm_led
	MAIN rs_pwm_led
	STACK_MAIN 1200
	COMPILE_FLAGS
	SRCS
		rs_pwm_led.cpp
	)
