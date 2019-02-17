
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v2
	LABEL rover
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	IO px4_io-v2_default

	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS6

	DRIVERS
		barometer/ms5611
		batt_smbus
		camera_capture
		camera_trigger
		distance_sensor # all available distance sensor drivers
		gps
		imu/l3gd20
		imu/lsm303d
		imu/mpu6000
		imu/mpu9250
		lights/rgbled
		magnetometer/hmc5883
		px4flow
		px4fmu
		px4io
		stm32
		stm32/adc
		stm32/tone_alarm

	MODULES
		camera_feedback
		commander
		dataman
		ekf2
		events
		gnd_att_control
		gnd_pos_control
		land_detector
		load_mon
		logger
		mavlink
		navigator
		sensors
		vmount

	SYSTEMCMDS
		bl_update
		#config
		#dumpfile
		#esc_calib
		hardfault_log
		#led_control
		mixer
		#motor_ramp
		#motor_test
		mtd
		#nshterm
		param
		perf
		pwm
		reboot
		#sd_bench
		top
		#topic_listener
		tune_control
		usb_connected
		ver
	)
