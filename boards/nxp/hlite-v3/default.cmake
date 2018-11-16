
px4_add_board(
	PLATFORM nuttx
	VENDOR nxp
	MODEL hlite-v3
	ARCH cortex-m4
	ROMFS
	ROMFSROOT px4fmu_common
	TESTING
	UAVCAN_INTERFACES 2

	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS4

	DRIVERS
		barometer # all available barometer drivers
		barometer/mpl3115a2
		batt_smbus
		blinkm
		camera_trigger
		differential_pressure # all available differential pressure drivers
		distance_sensor # all available distance sensor drivers
		gps
		kinetis
		kinetis/adc
		kinetis/tone_alarm
		#heater
		#imu # all available imu drivers
		imu/fxas21002c
		imu/fxos8701cq
		imu/l3gd20
		imu/mpu6000
		imu/mpu9250
		irlock
		magnetometer # all available magnetometer drivers
		mkblctrl
		oreoled
		pca9685
		#pwm_input # NOT Portable YET drivers
		pwm_out_sim
		px4flow
		px4fmu
		rc_input
		rgbled
		rgbled_pwm
		tap_esc
		telemetry # all available telemetry drivers
		#test_ppm # NOT Portable YET
		vmount

	MODULES
		attitude_estimator_q
		camera_feedback
		commander
		dataman
		ekf2
		events
		fw_att_control
		fw_pos_control_l1
		gnd_att_control
		gnd_pos_control
		gpio_led
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_pos_control
		navigator
		position_estimator_inav
		sensors
		uavcan
		vtol_att_control
		wind_estimator

	SYSTEMCMDS
		bl_update
		config
		dumpfile
		esc_calib
		#hardfault_log # Needs bbsrm
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
		tests # tests and test runner
		top
		topic_listener
		tune_control
		usb_connected
		ver

	EXAMPLES
		bottle_drop # OBC challenge
		fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		hwtest # Hardware test
		#matlab_csv_serial
		#publisher
		px4_mavlink_debug # Tutorial code from https://px4.io/dev/debug_values
		px4_simple_app # Tutorial code from https://px4.io/dev/px4_simple_app
		rover_steering_control # Rover example app
		segway
		#subscriber
		uuv_example_app

	)
