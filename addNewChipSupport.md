### Steps to add supports for a new chip ###
It is assumed that the chip is supported by the Nuttx.
This guide will provide information on porting the nuttx base layer and creating a custom board.

* Start with creating a folder with the board company name in `boards`.\
	`mkdir boards/myboard/`
* Create a folder with the name of the board in it.\
	`mkdir boards/myboard/myfc`
* Create a file called default.cmake in this folder.\
	`cd boards/myboard/myfc`
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
* Go to folder platforms/nuttx/cmake/Platform. This folder contains necessary compiler flags in different cmake files. If the architecture that you are working with is not available here, then you will have to create a file for your architecture.
* Now edit platforms/nuttx/cmake/px4_impl_os.cmake file to let the build system know where to look for the px4 base layer for your board. This edit will connect the choice of chip done in `defconfig` file with the respective chip's px4 base layer. Look for something like
	```
		elseif(CONFIG_ARCH_CHIP_S32K146)
			set(CHIP_MANUFACTURER "nxp")
			set(CHIP "s32k14x")
	```
* Add a similar entry for your chip if it doesn't already exist. Replace "nxp" whith whatever the folder name used for px4 base layer in platforms/nuttx/src/px4/ folder. For my case it is "rpi".
* Go ahead and create `rpi` folder in `platforms/nuttx/src/px4/`. Create a `CMakeLists.txt` file in that folder and copy following lines to that file\
	`add_subdirectory(${PX4_CHIP})`
* Now create two more folders with names `rpi_common` and `rp2040` inside `rpi`. Note that `rpi_common` will have files common to all the chips produced by Raspberrypi. While `rp2040` will have files specific to the rp2040 chip.
* Add an empty `CMakeLists.txt` file in `rp2040` folder for now. The contents of this file will be changed later as the need arises.
* Create `src` folder in `boards/myboard/myfc` folder. Put a `CMakeLists.txt` file in this `src` folder.
* Create `include` folder in `boards/myboard/myfc/nuttx-config` folder. Create a file `board.h` in this folder. This file will basically define all the necessary macros for the hardware for nuttx side of the code. For example, which pin is used for I2c, or what is the timer frequency, etc. For now, add following content to this file
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
* Some necessary macros can be copied from `plateforms/nuttx/NuttX/nuttx/boards/rp2040/raspberrypi-pico/include/board.h` file. This will help compiling the architecture specific files in nuttx.
* Add `board_config.h` file in `boards/myboard/myfc/src` folder. The `board_config.h` is similar to `board.h` file. However, it contains the necessary macros for the hardware for the PX4 side of the code. For now, add following content to this file
	```
		/**
		* @file board_config.h
		*
		* myboardmyfc internal definitions
		*/

		#pragma once

		/****************************************************************************************************
		* Included Files
		****************************************************************************************************/

		#include <px4_platform_common/px4_config.h>
		#include <nuttx/compiler.h>
		#include <stdint.h>

		__BEGIN_DECLS

		#ifndef __ASSEMBLY__

		#include <px4_platform_common/board_common.h>

		#endif /* __ASSEMBLY__ */

		__END_DECLS
	```
* Create `micro_hal.h` file in folders `platforms/nuttx/src/px4/rpi/rp2040/include/px4_arch` and `platforms/nuttx/src/px4/rpi/rpi_common/include/px4_arch`. This file mainly links a couple of px4 functions with their nuttx counterparts using macros for gpios, spi and i2c. These files also define some macros relating to cpu uuid and mfg uuid.
* Copy following content in file `platforms/nuttx/src/px4/rpi/rp2040/include/px4_arch/micro_hal.h`
	```
		#pragma once

		#include "../../../rpi_common/include/px4_arch/micro_hal.h"

		__BEGIN_DECLS

		#define PX4_SOC_ARCH_ID             PX4_SOC_ARCH_ID_UNUSED
		#define PX4_FLASH_BASE  RP2040_FLASH_BASE
		#define PX4_NUMBER_I2C_BUSES 2
		#define PX4_ADC_INTERNAL_TEMP_SENSOR_CHANNEL 4

		__END_DECLS
	```
* Copy following content in file `platforms/nuttx/src/px4/rpi/rpi_common/include/px4_arch`
	```
		#pragma once

		#include <px4_platform/micro_hal.h>

		__BEGIN_DECLS

		#define	RP2040_GPIO_PAD_MASK	0xff		// GPIO PAD register mask
		#define	RP2040_GPIO_PIN_MASK	0x1f00		// GPIO pin number mask
		#define	RP2040_GPIO_FUN_MASK	0x3e000		// GPIO pin function mask
		#define	RP2040_GPIO_OEN_MASK	0x40000		// GPIO output enable mask
		#define	RP2040_GPIO_OUT_MASK	0x80000		// GPIO output value mask

		#define	RP2040_PADS_BANK0_GPIO_RESET	0x56	// GPIO PAD register default value
		#define	RP2040_IO_BANK0_GPIO_CTRL_RESET	0x1f	// GPIO_CTRL register default value

		#include <rp2040_spi.h>
		#include <rp2040_i2c.h>
		#include <rp2040_gpio.h>

		// RP2040 doesn't really have a cpu register with unique id.
		// However, there is a function in pico-sdk which can provide
		// a device unique id from its flash which is 64 bits in length.
		// For now, a common device id will be used for all RP2040 based devices.
		// This can be done by defining a macro in the board's board_config.h file as shown below
		// #define BOARD_OVERRIDE_UUID "MYFC2040"	// must be of length 8

		#define PX4_CPU_UUID_BYTE_LENGTH                8
		#define PX4_CPU_UUID_WORD32_LENGTH              (PX4_CPU_UUID_BYTE_LENGTH/sizeof(uint32_t))

		/* The mfguid will be an array of bytes with
		* MSD @ index 0 - LSD @ index PX4_CPU_MFGUID_BYTE_LENGTH-1
		*
		* It will be converted to a string with the MSD on left and LSD on the right most position.
		*/
		#define PX4_CPU_MFGUID_BYTE_LENGTH              PX4_CPU_UUID_BYTE_LENGTH

		/* By not defining PX4_CPU_UUID_CORRECT_CORRELATION the following maintains the legacy incorrect order
		* used for selection of significant digits of the UUID in the PX4 code base.
		* This is done to avoid the ripple effects changing the IDs used on STM32 base platforms
		*/
		#if defined(PX4_CPU_UUID_CORRECT_CORRELATION)
		# define PX4_CPU_UUID_WORD32_UNIQUE_H            0 /* Least significant digits change the most */
		# define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
		# define PX4_CPU_UUID_WORD32_UNIQUE_L            2 /* Most significant digits change the least */
		#else
		/* Legacy incorrect ordering */
		# define PX4_CPU_UUID_WORD32_UNIQUE_H            2 /* Most significant digits change the least */
		# define PX4_CPU_UUID_WORD32_UNIQUE_M            1 /* Middle significant digits */
		# define PX4_CPU_UUID_WORD32_UNIQUE_L            0 /* Least significant digits change the most */
		#endif

		/*                                                  Separator    nnn:nnn:nnnn     2 char per byte           term */
		#define PX4_CPU_UUID_WORD32_FORMAT_SIZE         (PX4_CPU_UUID_WORD32_LENGTH-1+(2*PX4_CPU_UUID_BYTE_LENGTH)+1)
		#define PX4_CPU_MFGUID_FORMAT_SIZE              ((2*PX4_CPU_MFGUID_BYTE_LENGTH)+1)

		// #define px4_savepanic(fileno, context, length)  stm32_bbsram_savepanic(fileno, context, length)
		#define px4_savepanic(fileno, context, length)  (0)	// Turn off px4_savepanic for rp2040 as it is not implemented in nuttx

		#define PX4_BUS_OFFSET       1                  /* RP2040 buses are 0 based and adjustment is needed */
		#define px4_spibus_initialize(bus_num_0based)   rp2040_spibus_initialize(bus_num_0based)

		#define px4_i2cbus_initialize(bus_num_0based)   rp2040_i2cbus_initialize(bus_num_0based)
		#define px4_i2cbus_uninitialize(pdev)           rp2040_i2cbus_uninitialize(pdev)


		// This part of the code is specific to rp2040.
		// RP2040 does not have the gpio configuration process similar to stm or tiva devices.
		// There are multiple different registers which are required to be configured based on the function selection.
		// The pinset below can be defined using a 32-bit value where,
		// bits		Function
		// 0-7		PADS Bank 0 GPIO register. Take a look at rp2040 datasheet page 321.
		// 8-15		GPIO number. 0-29 is valid.
		// 16-23	GPIO function select
		// 24		Output enable
		// 25		Output value
		// 26-31	Unused
		// Take a look at k66 in nxp for referance.
		// void rp2040_pinconfig(uint32_t pinset);
		#define px4_arch_configgpio(pinset)             0 // rp2040_pinconfig(pinset)		// Implemented in io_pins/pin_config.c
		#define px4_arch_unconfiggpio(pinset)           0					// Needs to be implemented (can be done in io_pins)
		#define px4_arch_gpioread(pinset)               rp2040_gpio_get(pinset)			// Use gpio_get
		#define px4_arch_gpiowrite(pinset, value)       rp2040_gpio_put(pinset, value)		// Use gpio_put
		#define px4_arch_gpiosetevent(pinset,r,f,e,fp,a)  0					// Needs to be implemented (can be done in io_pins)

		// Following are quick defines to be used with the functions defined above
		// These defines create a bit-mask which is supposed to be used in the
		// functions defined above to set up gpios correctly.
		#define PX4_MAKE_GPIO_INPUT(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_PULLUP))
		#define PX4_MAKE_GPIO_OUTPUT_CLEAR(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_CLEAR))
		#define PX4_MAKE_GPIO_OUTPUT_SET(gpio) (((gpio) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_OUTPUT|GPIO_PUSHPULL|GPIO_SPEED_2MHz|GPIO_OUTPUT_SET))

		#define PX4_GPIO_PIN_OFF(def) (((def) & (GPIO_PORT_MASK | GPIO_PIN_MASK)) | (GPIO_INPUT|GPIO_FLOAT|GPIO_SPEED_2MHz))

		__END_DECLS
	```
* Note that a lot of things in these files might not exit in nuttx, for example rp2040_pinconfig function. The discussion regarding where to define such functions will be provided later.
