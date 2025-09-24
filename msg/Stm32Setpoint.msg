# Stm32Setpoint: setpoint and optional gains coming from STM32 via MAVLink

uint64 timestamp	# [us] time since system start

uint8  source_system	# MAVLink source system id
uint8  source_component	# MAVLink source component id

uint8  mode		# 0=stop, 1=open-loop duty [0..1], 2=closed-loop rpm

float32 setpoint	# open-loop: duty [0..1]; closed-loop: rpm

float32 kp
float32 ki
float32 kd

float32 out_min		# lower limit (e.g., 0)
float32 out_max		# upper limit (e.g., 1)

uint32 timeout_ms	# validity timeout from publish; 0 disables timeout handling here

