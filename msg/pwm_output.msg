uint64 timestamp			# time since system start (microseconds)
uint8 PWM_SERVO_MAX_CHANNELS = 16 	# Maximum number of PWM servo channels in the system. It's the same as the max channel in input_rc.msg.
uint8 channel_count			# number of channels actually being used
uint16[16] pulse_width			# pulse widths for each of the supported channels. In PWMOutBase::send_output_pwm, pwm value type is uint16_t.
