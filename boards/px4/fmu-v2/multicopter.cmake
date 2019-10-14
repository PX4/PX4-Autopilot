
px4_add_board(
	PLATFORM nuttx
	VENDOR px4
	MODEL fmu-v2
	LABEL multicopter
	TOOLCHAIN arm-none-eabi
	ARCHITECTURE cortex-m4
	ROMFSROOT px4fmu_common
	BOOTLOADER ${PX4_SOURCE_DIR}/ROMFS/px4fmu_common/extras/px4fmuv3_bl.bin
	IO px4_io-v2_default
	CONSTRAINED_FLASH
	#UAVCAN_INTERFACES 2

	SERIAL_PORTS
		GPS1:/dev/ttyS3
		TEL1:/dev/ttyS1
		TEL2:/dev/ttyS2
		TEL4:/dev/ttyS6

	DRIVERS
		adc
		barometer/ms5611
		#batt_smbus
		camera_capture
		camera_trigger
		distance_sensor # all available distance sensor drivers
		gps
		imu/l3gd20
		imu/lsm303d
		imu/mpu6000
		#imu/mpu9250
		irlock
		lights/rgbled
		magnetometer/hmc5883
		optical_flow/px4flow
		px4fmu
		px4io
		tone_alarm

	MODULES
		#attitude_estimator_q
		camera_feedback
		commander
		dataman
		ekf2
		events
		land_detector
		landing_target_estimator
		load_mon
		#local_position_estimator
		logger
		mavlink
		mc_att_control
		mc_pos_control
		navigator
		sensors
		vmount

	SYSTEMCMDS
		bl_update
		#config
		#dumpfile
		#esc_calib
		hardfault_log
		#i2cdetect
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
		work_queue

	)
