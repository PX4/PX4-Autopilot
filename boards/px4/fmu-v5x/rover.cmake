
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v5x
	LABEL rover
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m7
	ROMFSROOT px4fmu_common
	IO px4_io-v2_default
	UAVCAN_INTERFACES 2

	SERIAL_PORTS
		GPS1:/dev/ttyS1
		TEL1:/dev/ttyS6
		TEL2:/dev/ttyS4
		TEL3:/dev/ttyS2
		GPS2:/dev/ttyS0

	DRIVERS
		barometer # all available barometer drivers
		batt_smbus
		camera_capture
		camera_trigger
		distance_sensor # all available distance sensor drivers
		gps
		#heater
		imu/adis16448
		imu/adis16497
		#imu # all available imu drivers
# TBD		imu/bmi088 - needs bus selection
# TBD		imu/ism330dlc - needs bus selection
		imu/mpu6000
		irlock
		lights/blinkm
		lights/oreoled
		lights/pca8574
		lights/rgbled
		lights/rgbled_ncp5623c
		magnetometer # all available magnetometer drivers
		#md25
		mkblctrl
		optical_flow # all available optical flow drivers
		pca9685
		power_monitor/ina226
		#protocol_splitter
		pwm_input
		pwm_out_sim
		px4fmu
		px4io
		rc_input
		roboclaw
		stm32
		stm32/adc
		stm32/tone_alarm
		telemetry # all available telemetry drivers
		tone_alarm
		uavcan

	MODULES
		camera_feedback
		commander
		dataman
		ekf2
		events
		land_detector
		load_mon
		logger
		mavlink
		navigator
		rover_pos_control
		sensors
		vmount

	SYSTEMCMDS
		bl_update
		config
		dmesg
		dumpfile
		esc_calib
		hardfault_log
		i2cdetect
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
