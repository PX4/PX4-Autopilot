
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v5
	LABEL rover
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
		distance_sensor # all available distance sensor drivers
		gps
		imu/bmi055
		imu/mpu6000
		lights/pca8574
		lights/rgbled
		lights/rgbled_ncp5623c
		lights/rgbled_pwm
		magnetometer # all available magnetometer drivers
		#md25
		mkblctrl
		pca9685
		pmw3901
		pwm_input
		pwm_out_sim
		px4flow
		px4fmu
		px4io
		rc_input
		roboclaw
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
