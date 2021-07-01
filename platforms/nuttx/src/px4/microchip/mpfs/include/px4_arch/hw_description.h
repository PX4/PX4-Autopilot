/****************************************************************************
 *
 *   Copyright (C) 2021 Technology Innovation Institute. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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

#pragma once


#include <stdint.h>

#include <px4_platform_common/constexpr_util.h>

#include <mpfs_gpio.h>

/*
 * Timers
 */

namespace Timer
{
enum Timer {
	FTM0 = 0,
};
enum Channel {
	Channel0 = 0,
	Channel1,
	Channel2,
	Channel3,
	Channel4,
	Channel5,
};
struct TimerChannel {
	Timer timer;
	Channel channel;
};
}

namespace GPIO
{
enum Bank {
	BankInvalid = 0,
	Bank0,
	Bank1,
	Bank2,
};
enum Pin {
	Pin0 = 0,
	Pin1,
	Pin2,
	Pin3,
	Pin4,
	Pin5,
	Pin6,
	Pin7,
	Pin8,
	Pin9,
	Pin10,
	Pin11,
	Pin12,
	Pin13,
	Pin14,
	Pin15,
	Pin16,
	Pin17,
	Pin18,
	Pin19,
	Pin20,
	Pin21,
	Pin22,
	Pin23,
	Pin24,
	Pin25,
	Pin26,
	Pin27,
	Pin28,
	Pin29,
	Pin30,
	Pin31,
};

struct GPIOPin {
	Bank bank;
	Pin pin;
};
}

static inline constexpr uint32_t getGPIOBank(GPIO::Bank bank)
{
	switch (bank) {
	case GPIO::Bank0: return GPIO_BANK0;

	case GPIO::Bank1: return GPIO_BANK1;

	case GPIO::Bank2: return GPIO_BANK2;

	default: break;
	}

	return 0;
}

static inline constexpr uint32_t getGPIOPin(GPIO::Pin pin)
{
	switch (pin) {
	case GPIO::Pin0: return GPIO_PIN0;

	case GPIO::Pin1: return GPIO_PIN1;

	case GPIO::Pin2: return GPIO_PIN2;

	case GPIO::Pin3: return GPIO_PIN3;

	case GPIO::Pin4: return GPIO_PIN4;

	case GPIO::Pin5: return GPIO_PIN5;

	case GPIO::Pin6: return GPIO_PIN6;

	case GPIO::Pin7: return GPIO_PIN7;

	case GPIO::Pin8: return GPIO_PIN8;

	case GPIO::Pin9: return GPIO_PIN9;

	case GPIO::Pin10: return GPIO_PIN10;

	case GPIO::Pin11: return GPIO_PIN11;

	case GPIO::Pin12: return GPIO_PIN12;

	case GPIO::Pin13: return GPIO_PIN13;

	case GPIO::Pin14: return GPIO_PIN14;

	case GPIO::Pin15: return GPIO_PIN15;

	case GPIO::Pin16: return GPIO_PIN16;

	case GPIO::Pin17: return GPIO_PIN17;

	case GPIO::Pin18: return GPIO_PIN18;

	case GPIO::Pin19: return GPIO_PIN19;

	case GPIO::Pin20: return GPIO_PIN20;

	case GPIO::Pin21: return GPIO_PIN21;

	case GPIO::Pin22: return GPIO_PIN22;

	case GPIO::Pin23: return GPIO_PIN23;

	case GPIO::Pin24: return GPIO_PIN24;

	case GPIO::Pin25: return GPIO_PIN25;

	case GPIO::Pin26: return GPIO_PIN26;

	case GPIO::Pin27: return GPIO_PIN27;

	case GPIO::Pin28: return GPIO_PIN28;

	case GPIO::Pin29: return GPIO_PIN29;

	case GPIO::Pin30: return GPIO_PIN30;

	case GPIO::Pin31: return GPIO_PIN31;
	}

	return 0;
}


namespace SPI
{
enum class Bus {
	SPI0 = 1,
	SPI1 = 2,
};

using CS = GPIO::GPIOPin; ///< chip-select pin
using DRDY = GPIO::GPIOPin; ///< data ready pin

struct bus_device_external_cfg_t {
	CS cs_gpio;
	DRDY drdy_gpio;
};

} // namespace SPI
