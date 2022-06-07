/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file apa102.cpp
 *
 * Driver for the the apa102 class of RGB LED drivers.
 * this driver is based on the PX4 led driver
 *
 */

#include "apa102.hpp"

// Constructor
APA102::APA102(unsigned int number_of_packages, int bus, uint32_t device, int bus_frequency, spi_mode_e spi_mode) :
	SPI(DRV_DEVTYPE_UNUSED, MODULE_NAME, bus, device, spi_mode, bus_frequency),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_number_of_packages(number_of_packages)
{
}

// Deconstructor
APA102::~APA102()
{
	//TODO: deinit SPI bus
	//apa102_deinit();
}

// Init
// This runs after calling the constructor
int APA102::init()
{
	// Init SPI bus
	int ret = SPI::init();

	if (ret != OK) {
		printf("SPI::init() failed\n");
		DEVICE_DEBUG("SPI init failed");
		return -EIO;
	}

	// Fill buffer with zeros
	for (uint8_t i = 0; i < (_number_of_packages * 4) + 8; i++) {
		buf[i] = 0x00;
	}

	// APA102LEDData is just the data format for the BRG LED
	_leds = new apa102::APA102LEDData [_number_of_packages];

	if (_leds == nullptr) {
		return PX4_ERROR;
	}

	ScheduleNow();
	return OK;
}

//
int APA102::task_spawn(int argc, char *argv[])
{
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;
	unsigned int number_of_packages = BOARD_MAX_LEDS;

	while ((ch = px4_getopt(argc, argv, "n:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'n':
			number_of_packages = atoi(myoptarg) + 1;

			if (number_of_packages > BOARD_MAX_LEDS) {
				number_of_packages = BOARD_MAX_LEDS;
				PX4_INFO("Number of packages can not exceed BOARD_MAX_LEDS");
			}

			break;

		default:
			print_usage("unrecognized option");
			return 1;
		}
	}

	printf("Number of packages: %d\n", number_of_packages);
	APA102 *instance = new APA102(number_of_packages, 1, 0, 4000000, SPIDEV_MODE0);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int APA102::print_status()
{

	PX4_INFO("Controlling %i LEDs", _number_of_packages);

	return 0;
}

int APA102::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
This module is responsible for driving interfasing to the Neopixel Serial LED

### Examples
It is typically started with:
$ apa102 -n 8
To drive all available leds.
)DESCR_STR");

PRINT_MODULE_USAGE_NAME("newpixel", "driver");
PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
return 0;
}

int APA102::custom_command(int argc, char *argv[])
{
   return print_usage("unrecognized option");
}

/**
 * Main loop function
 * This will run periodically by the scheduler
 */
void APA102::Run()
{
  if (should_exit()) {
      ScheduleClear();
      exit_and_cleanup();
      return;
    }

    // Structure for uORB data
	LedControlData led_control_data;

	// Get LED data from uORB (led_control)
	if (_led_controller.update(led_control_data) == 1) {

		// Loop through each LED
	    for (unsigned int led = 1; led < math::min(_number_of_packages, arraySize(led_control_data.leds)); led++) {

	    	// Set brightness
	        uint8_t brightness = led_control_data.leds[led].brightness;
	        
	        /* Brightness is not 0-255, it is 5 bit, so 0-31. 256/32=8 */
	        brightness = brightness / 8;
	        //printf("BRIGHTNESS: %d\n", brightness);
	        //printf("arraySize(%d)\n", arraySize(led_control_data.leds));

	        // Use data from uORB to set specific APA102LEDData fields
	        switch (led_control_data.leds[led].color) {
	          case led_control_s::COLOR_RED:
	            _leds[led].R() = 255; _leds[led].G() = 0; _leds[led].B() = 0;
	            break;

	          case led_control_s::COLOR_DIM_RED:
	            _leds[led].R() = 16; _leds[led].G() = 0; _leds[led].B() = 0;
	            break;

	          case led_control_s::COLOR_GREEN:
	            _leds[led].R() = 0; _leds[led].G() = 255; _leds[led].B() = 0;
	            break;

	          case led_control_s::COLOR_BLUE:
	            _leds[led].R() = 0; _leds[led].G() = 0; _leds[led].B() = 255;
	            break;

	          case led_control_s::COLOR_AMBER: //make it the same as yellow
	          case led_control_s::COLOR_YELLOW:
	            _leds[led].R() = 255; _leds[led].G() = 255; _leds[led].B() = 0;
	            break;

	          case led_control_s::COLOR_PURPLE:
	            _leds[led].R() = 255; _leds[led].G() = 0; _leds[led].B() = 255;
	            break;

	          case led_control_s::COLOR_CYAN:
	            _leds[led].R() = 0; _leds[led].G() = 255; _leds[led].B() = 255;
	            break;

	          case led_control_s::COLOR_WHITE:
	            _leds[led].R() = 255; _leds[led].G() = 255; _leds[led].B() = 255;
	            break;

	          default: // led_control_s::COLOR_OFF
	            _leds[led].R() = 0; _leds[led].G() = 0; _leds[led].B() = 0;
	            break;
	        } // end switch

			/* APA102 Frame
			 * 0x000000000 [start frame] (this is already done in init)
			 * 0xE0 + [brightness]
			 * 0xXXXXXX [bgr]
			 * 0xFFFFFFFF [end frame]
			 */

			for(uint8_t i = 1; i < _number_of_packages; i++)
			{
				buf[4+(4*(i-1))] = 0b11100000 + brightness;
				buf[5+(4*(i-1))] = _leds[i].B();
				buf[6+(4*(i-1))] = _leds[i].G();
				buf[7+(4*(i-1))] = _leds[i].R();
			} // end FOR loop

			// Fill with end frame
			buf[8+(4*_number_of_packages)] = 0xFF;
			buf[9+(4*_number_of_packages)] = 0xFF;
			buf[10+(4*_number_of_packages)] = 0xFF;
			buf[11+(4*_number_of_packages)] = 0xFF;

		} // end FOR loop

		transfer(buf, rbuf, (_number_of_packages * 4) + 8);

	} // end IF statement

	ScheduleDelayed(_led_controller.maximum_update_interval());
} // end FUNCTION

// Main function
// This runs when calling the command on the command line
extern "C" __EXPORT int apa102_main(int argc, char *argv[])
{
	// This calls task_spawn
  return APA102::main(argc, argv);
}
