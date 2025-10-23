/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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

#include <s32k3xx_pin.h>

#include <px4_platform_common/constexpr_util.h>

/*
 * Timers
 */

namespace Timer
{


// Just to keep def extract_timer(line): happy
enum Timer {
	EMIOS0_Channel0,
	EMIOS0_Channel1,
	EMIOS0_Channel2,
	EMIOS0_Channel3,
	EMIOS0_Channel4,
	EMIOS0_Channel5,
	EMIOS0_Channel6,
	EMIOS0_Channel7,
	EMIOS1_Channel0,
	EMIOS1_Channel1,
	EMIOS1_Channel2,
	EMIOS1_Channel3,
	EMIOS1_Channel4,
	EMIOS1_Channel5,
	EMIOS1_Channel6,
	EMIOS1_Channel7,
	EMIOS2_Channel0,
	EMIOS2_Channel1,
	EMIOS2_Channel2,
	EMIOS2_Channel3,
	EMIOS2_Channel4,
	EMIOS2_Channel5,
	EMIOS2_Channel6,
	EMIOS2_Channel7,
};
enum Channel {
	Channel0 = 0,
	Channel1,
	Channel2,
	Channel3,
	Channel4,
	Channel5,
	Channel6,
	Channel7,
};
struct TimerChannel {
	Timer timer;
	Channel channel;
};
}

static inline constexpr uint32_t timerBaseRegister(Timer::Timer timer)
{
	switch (timer) {
	case Timer::EMIOS0_Channel0:
	case Timer::EMIOS0_Channel1:
	case Timer::EMIOS0_Channel2:
	case Timer::EMIOS0_Channel3:
	case Timer::EMIOS0_Channel4:
	case Timer::EMIOS0_Channel5:
	case Timer::EMIOS0_Channel6:
	case Timer::EMIOS0_Channel7:
		return S32K3XX_EMIOS0_BASE;



	case Timer::EMIOS1_Channel0:
	case Timer::EMIOS1_Channel1:
	case Timer::EMIOS1_Channel2:
	case Timer::EMIOS1_Channel3:
	case Timer::EMIOS1_Channel4:
	case Timer::EMIOS1_Channel5:
	case Timer::EMIOS1_Channel6:
	case Timer::EMIOS1_Channel7:
		return S32K3XX_EMIOS1_BASE;

	case Timer::EMIOS2_Channel0:
	case Timer::EMIOS2_Channel1:
	case Timer::EMIOS2_Channel2:
	case Timer::EMIOS2_Channel3:
	case Timer::EMIOS2_Channel4:
	case Timer::EMIOS2_Channel5:
	case Timer::EMIOS2_Channel6:
	case Timer::EMIOS2_Channel7:
		return S32K3XX_EMIOS2_BASE;
	}

	return 0;
}

/*
 * GPIO
 */

namespace GPIO
{
enum Port {
	PortInvalid = 0,
	PortA,
	PortB,
	PortC,
	PortD,
	PortE,
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
	Port port;
	Pin pin;
};
}

static inline constexpr uint32_t getGPIOPort(GPIO::Port port)
{
	switch (port) {
	case GPIO::PortA: return PIN_PORTA;

	case GPIO::PortB: return PIN_PORTB;

	case GPIO::PortC: return PIN_PORTC;

	case GPIO::PortD: return PIN_PORTD;

	case GPIO::PortE: return PIN_PORTE;

	default: break;
	}

	return 0;
}

static inline constexpr uint32_t getGPIOPin(GPIO::Pin pin)
{
	switch (pin) {
	case GPIO::Pin0: return PIN0;

	case GPIO::Pin1: return PIN1;

	case GPIO::Pin2: return PIN2;

	case GPIO::Pin3: return PIN3;

	case GPIO::Pin4: return PIN4;

	case GPIO::Pin5: return PIN5;

	case GPIO::Pin6: return PIN6;

	case GPIO::Pin7: return PIN7;

	case GPIO::Pin8: return PIN8;

	case GPIO::Pin9: return PIN9;

	case GPIO::Pin10: return PIN10;

	case GPIO::Pin11: return PIN11;

	case GPIO::Pin12: return PIN12;

	case GPIO::Pin13: return PIN13;

	case GPIO::Pin14: return PIN14;

	case GPIO::Pin15: return PIN15;

	case GPIO::Pin16: return PIN16;

	case GPIO::Pin17: return PIN17;

	case GPIO::Pin18: return PIN18;

	case GPIO::Pin19: return PIN19;

	case GPIO::Pin20: return PIN20;

	case GPIO::Pin21: return PIN21;

	case GPIO::Pin22: return PIN22;

	case GPIO::Pin23: return PIN23;

	case GPIO::Pin24: return PIN24;

	case GPIO::Pin25: return PIN25;

	case GPIO::Pin26: return PIN26;

	case GPIO::Pin27: return PIN27;

	case GPIO::Pin28: return PIN28;

	case GPIO::Pin29: return PIN29;

	case GPIO::Pin30: return PIN30;

	case GPIO::Pin31: return PIN31;
	}

	return 0;
}

static inline constexpr uint32_t getIMCR(uint16_t imcr)
{
	return IMCR(imcr);
}

namespace SPI
{
// SPI3 is ignored only used for FS26

enum class Bus {
	SPI0 = 1,
	SPI1,
	SPI2,
	SPI3,
	SPI4,
	SPI5,
};

using CS = GPIO::GPIOPin;
using DRDY = uint32_t;

#define DRDYInvalid 0

struct bus_device_external_cfg_t {
	CS cs_gpio;
	DRDY drdy_gpio;
};

} // namespace SPI
