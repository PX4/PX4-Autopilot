
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v5
	LABEL fixedwing
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	IO px4_io-v2_default
	UAVCAN_INTERFACES 2

	SERIAL_PORTS
		GPS1:/dev/ttyS0
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS3

	DRIVERS
		barometer # all available barometer drivers
		batt_smbus
		camera_capture
		camera_trigger
		differential_pressure # all available differential pressure drivers
		distance_sensor # all available distance sensor drivers
		gps
		imu/adis16448
		imu/bmi055
		imu/mpu6000
		lights/rgbled
		lights/rgbled_ncp5623c
		lights/rgbled_pwm
		magnetometer # all available magnetometer drivers
		pwm_input
		pwm_out_sim
		px4fmu
		px4io
		rc_input
		stm32
		stm32/adc
		stm32/armv7-m_dcache
		stm32/tone_alarm
		telemetry # all available telemetry drivers
		uavcan

	MODULES
		camera_feedback
		commander
		dataman
		ekf2
		events
		fw_att_control
		fw_pos_control_l1
		land_detector
		load_mon
		logger
		mavlink
		navigator
		sensors
		vmount
		wind_estimator

	SYSTEMCMDS
		bl_update
		config
		dumpfile
		esc_calib
		hardfault_log
		led_control
		mixer
		motor_ramp
		motor_test
		mtd
		nshterm
		param
		perf
		pwm
		reboot
		reflect
		sd_bench
		shutdown
		top
		topic_listener
		tune_control
		usb_connected
		ver
	)
