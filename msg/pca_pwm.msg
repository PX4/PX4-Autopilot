uint64 timestamp				# time since system start (microseconds)
uint16 pwm_period				# PWM period in microseconds (us) [roof((1s/1526hz)*10^7us/s)=656us:floor((1s/24hz)*10^7us/s)=41666us]
uint16[16] pulse_width				# pulse width in microseconds (us) [656:41666]

# TOPICS pca_pwm
