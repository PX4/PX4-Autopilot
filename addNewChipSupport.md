### Steps to add supports for a new chip ###
It is assumed that the chip is supported by the Nuttx.
This guide will provide information on porting the nuttx base layer and creating a custom board.

* Start with creating a folder with the board company name in PX4-Autopilot/boards.\
	`mkdir PX4-Autopilot/boards/myboard/`
* Create a folder with the name of the board in it.\
	`mkdir PX4-Autopilot/boards/myboard/myfc`
* Create a file called default.cmake in this folder.\
	`cd PX4-Autopilot/boards/myboard/myfc`
	`touch default.cmake`
* This file lets the PX4 build system know almost everything about how to build PX4 for this board. Copy following content in the default.cmake file. Note that most of the components are commented as they are not required in the initial build.
	```
		px4_add_board(
		PLATFORM nuttx
		VENDOR myboard
		MODEL myfc
		LABEL default
		TOOLCHAIN arm-none-eabi
		ARCHITECTURE cortex-m0plus
		CONSTRAINED_MEMORY
		ROMFSROOT px4fmu_common
		CONSTRAINED_FLASH
		SERIAL_PORTS
		# 	TEL2:/dev/ttyS1
		# 	URT6:/dev/ttyS2
		DRIVERS
		# 	adc/board_adc
		# 	#barometer # all available barometer drivers
		# 	barometer/bmp280
		# 	#batt_smbus
		# 	#camera_trigger
		# 	#differential_pressure # all available differential pressure drivers
		# 	#distance_sensor # all available distance sensor drivers
		# 	dshot
		# 	gps
		# 	imu/invensense/icm20602
		# 	imu/invensense/mpu6000
		# 	#irlock
		# 	lights/rgbled
		# 	#magnetometer # all available magnetometer drivers
		# 	magnetometer/hmc5883
		# 	#optical_flow # all available optical flow drivers
		# 	osd
		# 	#pca9685
		# 	#pwm_input
		# 	#pwm_out_sim
		# 	pwm_out
		# 	rc_input
		# 	#telemetry # all available telemetry drivers
		# 	telemetry/frsky_telemetry
		MODULES
		# 	#airspeed_selector
		# 	attitude_estimator_q
		# 	battery_status
		# 	#camera_feedback
		# 	commander
		# 	dataman
		# 	ekf2
		# 	#esc_battery
		# 	events
		# 	flight_mode_manager
		# 	#fw_att_control
		# 	#fw_pos_control_l1
		# 	gyro_calibration
		# 	#gyro_fft
		# 	land_detector
		# 	#landing_target_estimator
		# 	load_mon
		# 	#local_position_estimator
		# 	logger
		# 	mavlink
		# 	mc_att_control
		# 	mc_hover_thrust_estimator
		# 	mc_pos_control
		# 	mc_rate_control
		# 	#micrortps_bridge
		# 	navigator
		# 	rc_update
		# 	#rover_pos_control
		# 	sensors
		# 	#sih
		# 	#temperature_compensation
		# 	#uuv_att_control
		# 	#uuv_pos_control
		# 	#vmount
		# 	#vtol_att_control
		SYSTEMCMDS
		# 	#bl_update
		# 	dmesg
		# 	dumpfile
		# 	esc_calib
		# 	#gpio
		# 	hardfault_log
		# 	i2cdetect
		# 	led_control
		# 	#mft
		# 	mixer
		# 	#motor_ramp
		# 	motor_test
		# 	#mtd
		# 	nshterm
		# 	param
		# 	perf
		# 	pwm
		# 	reboot
		# 	reflect
		# 	sd_bench
		# 	#serial_test
		# 	#system_time
		# 	top
		# 	#topic_listener
		# 	tune_control
		# 	uorb
		# 	usb_connected
		# 	ver
		# 	work_queue
		)
	```
* Now create a firmware.prototype file (Not sure bout the use of this file) with following content in it.
	```
		{
			"board_id": 42,
			"magic": "OMNIBUSF4SD",
			"description": "Firmware for the OmnibusF4SD board",
			"image": "",
			"build_time": 0,
			"summary": "PX4/OmnibusF4",
			"version": "0.1",
			"image_size": 0,
			"image_maxsize": 1032192,
			"git_identity": "",
			"board_revision": 0
		}
	```
* Create a folder with name `nuttx-config`. This folder will contain everything related to building Nuttx. Two more folders are required here with names `nsh` and `scripts` respectively. The `nsh` folder will contain `defconfig` file for building Nuttx. The `scripts` folder will contain `scripts.ld` file which is the arm linker file defining memory regions and stuff. Both of these files can probably be found in the Nuttx's boards folder for a board using the same chip that you might be using.
* Go to folder PX4-Autopilot/platforms/nuttx/cmake/Platform. This folder contains necessary compiler flags in different cmake files. If the architecture that you are working with is not available here, then you will have to create a file for your architecture.
* Now edit PX4-Autopilot/platforms/nuttx/cmake/px4_impl_os.cmake file to let the build system know where to look for the px4 base layer for your board. This edit will connect the choice of chip done in `defconfig` file with the respective chip's px4 base layer. Look for something like
	```
		elseif(CONFIG_ARCH_CHIP_S32K146)
			set(CHIP_MANUFACTURER "nxp")
			set(CHIP "s32k14x")
	```
* Add a similar entry for your chip if it doesn't already exist. Replace "nxp" whith whatever the folder name used for px4 base layer in PX4-Autopilot/platforms/nuttx/src/px4/ folder. For my case it is "rpi".
* Go ahead and create `rpi` folder in `PX4-Autopilot/platforms/nuttx/src/px4/`. Create a `CMakeLists.txt` file in that folder and copy following lines to that file\
	`add_subdirectory(${PX4_CHIP})`
* Now create two more folders with names `rpi_common` and `rp2040` inside `rpi`. Note that `rpi_common` will have files common to all the chips produced by Raspberrypi. While `rp2040` will have files specific to the rp2040 chip.
* Add an empty `CMakeLists.txt` file in `rp2040` folder for now. The contents of this file will be changed later as the need arises.
* Create `src` folder in `PX4-Autopilot/boards/myboard/myfc` folder. Put a `CMakeLists.txt` file in this `src` folder.
* Create `include` folder in `PX4-Autopilot/boards/myboard/myfc/nuttx-config` folder. Create a file `board.h` in this folder. This file will basically define all the necessary macros for the hardware. For example, which pin is used for I2c, or what is the timer frequency, etc. For now, add following content to this file
	```
		#ifndef __CONFIG_MYBOARDMYFC_INCLUDE_BOARD_H
		#define __CONFIG_MYBOARDMYFC_INCLUDE_BOARD_H

		/************************************************************************************
		* Included Files
		************************************************************************************/
		#include <nuttx/config.h>

		#ifndef __ASSEMBLY__
		# include <stdint.h>
		#endif

		#endif  /* __CONFIG_MYBOARDMYFC_INCLUDE_BOARD_H */
	```
