
px4_add_board(
	PLATFORM nuttx
	VENDOR holybro
	MODEL kakutef7
	LABEL default
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common

	SERIAL_PORTS
		TEL1:/dev/ttyS0 # UART1
		TEL2:/dev/ttyS1 # UART2
#		TEL3:/dev/ttyS2 # UART3 (currently NuttX console)
		GPS1:/dev/ttyS3 # UART4
		RC:/dev/ttyS4 # UART6
		# /dev/ttyS5: UART7 (ESC telemetry)

	DRIVERS
		adc
		barometer/bmp280
		dshot
		gps
		imu/mpu6000
		magnetometer
		optical_flow/px4flow
		pwm_out_sim
		px4fmu
		rc_input
		telemetry
		tone_alarm
		osd

	MODULES
		attitude_estimator_q
		commander
		dataman
		#ekf2
		events
		land_detector
		load_mon
		logger
		mavlink
		mc_att_control
		mc_pos_control
		navigator
		sensors

	SYSTEMCMDS
		bl_update
		config
		dmesg
		dumpfile
		esc_calib
		hardfault_log
		led_control
		mixer
		#motor_ramp
		motor_test
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		sd_bench
		shutdown
		top
		tune_control
		topic_listener
		usb_connected
		ver
		work_queue

	)
