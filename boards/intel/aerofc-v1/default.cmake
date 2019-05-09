
px4_add_board(
	PLATFORM nuttx
	VENDOR intel
	MODEL aerofc-v1
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common

	SERIAL_PORTS
		GPS1:/dev/ttyS5
		TEL1:/dev/ttyS3
		TEL2:/dev/ttyS1

	DRIVERS
		barometer/ms5611
		#camera_trigger
		#differential_pressure # all available differential pressure drivers
		distance_sensor
		gps
		#irlock
		imu/mpu9250
		#magnetometer # all available magnetometer drivers
		magnetometer/hmc5883
		magnetometer/ist8310
		pwm_out_sim
		#px4flow
		px4fmu
		rc_input
		stm32
		tap_esc
		#telemetry # all available telemetry drivers
		#uavcan

	MODULES
		attitude_estimator_q
		#camera_feedback
		commander
		dataman
		ekf2
		events
		#fw_att_control
		#fw_pos_control_l1
		#gnd_att_control
		#gnd_pos_control
		land_detector
		landing_target_estimator
		load_mon
		local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_pos_control
		navigator
		sensors
		sih
		vmount
		#vtol_att_control
		#wind_estimator

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
		#tests # tests and test runner
		top
		#topic_listener
		tune_control
		ver

	EXAMPLES
		#bottle_drop # OBC challenge
		#fixedwing_control # Tutorial code from https://px4.io/dev/example_fixedwing_control
		#hwtest # Hardware test
		#matlab_csv_serial
		#px4_mavlink_debug # Tutorial code from http://dev.px4.io/en/debug/debug_values.html
		#px4_simple_app # Tutorial code from http://dev.px4.io/en/apps/hello_sky.html
		#rover_steering_control # Rover example app
		#segway
		#uuv_example_app

	)
