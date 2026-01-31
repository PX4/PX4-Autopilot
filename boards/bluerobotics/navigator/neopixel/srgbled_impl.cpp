/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file srgbled_impl.cpp
 *
 * Implementation of functions to control NeoPixel LEDs on the BlueRobotics Navigator for the neopixel driver.
 *
 * Definitions:
 * - BOARD_HAS_N_S_RGB_LED: Number of NeoPixel LEDs on the Navigator.
 * - BOARD_RGB_SPI_BUS: SPI bus connected to the NeoPixel data line.
 * - BOARD_RGB_SPI_FREQ: SPI bus frequency for the NeoPixel connection.
 */

#include <board_config.h>
#include <drivers/drv_neopixel.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/sem.h>

#if defined(BOARD_HAS_N_S_RGB_LED)

#if !defined(BOARD_RGB_SPI_BUS)
#  error  BOARD_RGB_SPI_BUS must be defined to use the NeoPixel driver implementation for the BlueRobotics Navigator
#endif
#if !defined(BOARD_RGB_SPI_FREQ)
#  error  BOARD_RGB_SPI_FREQ must be defined to use the NeoPixel driver implementation for the BlueRobotics Navigator
#endif

#define LED_T0 0b11000000
#define LED_T1 0b11110000

class NavigatorLED : public device::SPI
{
public:
	NavigatorLED();
	~NavigatorLED() override = default;

	int init();
	int write(uint8_t red, uint8_t green, uint8_t blue);
	int deinit();

protected:
	void _setup_data(uint8_t red, uint8_t green, uint8_t blue);

private:
	uint8_t _data[24];
	px4_sem_t _sem;
};

NavigatorLED::NavigatorLED() :
	// By default we don't use the CS line and MODE must be 0 for compatibility with Raspberry Pi
	SPI(DRV_DEVTYPE_UNUSED, "navigator-neopixel", BOARD_RGB_SPI_BUS, 0, SPIDEV_MODE0, BOARD_RGB_SPI_FREQ)
{
	px4_sem_init(&_sem, 0, 1);
}

int NavigatorLED::init()
{
	int ret = SPI::init();

	if (ret != PX4_OK) {
		PX4_ERR("Neopixel SPI init failed");
		return ret;
	}

	return PX4_OK;
}

void NavigatorLED::_setup_data(uint8_t red, uint8_t green, uint8_t blue)
{
	for (uint8_t i = 0; i < 8; i++) {
		_data[i] = (green & (1 << (7 - i))) ? LED_T1 : LED_T0;
	}

	for (uint8_t i = 0; i < 8; i++) {
		_data[8 + i] = (red & (1 << (7 - i))) ? LED_T1 : LED_T0;
	}

	for (uint8_t i = 0; i < 8; i++) {
		_data[16 + i] = (blue & (1 << (7 - i))) ? LED_T1 : LED_T0;
	}
}

int NavigatorLED::write(uint8_t red, uint8_t green, uint8_t blue)
{
	_setup_data(red, green, blue);

	px4_sem_wait(&_sem);

	int ret = transfer(_data, nullptr, sizeof(_data));

	px4_sem_post(&_sem);

	if (ret != PX4_OK) {
		PX4_ERR("Failed to write data to NeoPixel %d", ret);
	}

	return ret;
}

int NavigatorLED::deinit()
{
	PX4_INFO("Neopixel deinitialized");
	return PX4_OK;
}

// Neopixel driver impl

NavigatorLED navigator_led;

int neopixel_init(neopixel::NeoLEDData *led_data, int number_of_packages)
{
	return navigator_led.init();
}

int neopixel_write(neopixel::NeoLEDData *led_data, int number_of_packages)
{
	return navigator_led.write(led_data->R(), led_data->G(), led_data->B());
}

int neopixel_deinit()
{
	return navigator_led.deinit();
}
#endif // defined(BOARD_HAS_N_S_RGB_LED)
