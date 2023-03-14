/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *  notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *  notice, this list of conditions and the following disclaimer in
 *  the documentation and/or other materials provided with the
 *  distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *  used to endorse or promote products derived from this software
 *  without specific prior written permission.
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

#include "hardware/imxrt_flexpwm.h"

#include <px4_platform_common/constexpr_util.h>

#include <board_config.h>
#if !defined(CONFIG_ARCH_CHIP_MIMXRT1176DVMAA)
# error "This code has only been validated with IMXRT1176. Make sure it is correct before using it on another board."
#endif

/*
 * PWM
 */

namespace PWM
{
enum FlexPWM {
	FlexPWM1 = 0,
	FlexPWM2,
	FlexPWM3,
	FlexPWM4,
};

enum FlexPWMModule {
	PWM1_PWM_A = 0,
	PWM1_PWM_B,
	PWM1_PWM_X,

	PWM2_PWM_A,
	PWM2_PWM_B,
	PWM2_PWM_X,

	PWM3_PWM_A,
	PWM3_PWM_B,
	PWM3_PWM_X,

	PWM4_PWM_A,
	PWM4_PWM_B,
	PWM4_PWM_X,
};

enum FlexPWMSubmodule {
	Submodule0 = 0,
	Submodule1,
	Submodule2,
	Submodule3,
};

struct FlexPWMConfig {
	FlexPWMModule module;
	FlexPWMSubmodule submodule;
};
}

static inline constexpr uint32_t getFlexPWMBaseRegister(PWM::FlexPWM pwm)
{
	switch (pwm) {
	case PWM::FlexPWM1: return IMXRT_FLEXPWM1_BASE;

	case PWM::FlexPWM2: return IMXRT_FLEXPWM2_BASE;

	case PWM::FlexPWM3: return IMXRT_FLEXPWM3_BASE;

	case PWM::FlexPWM4: return IMXRT_FLEXPWM4_BASE;
	}

	return 0;
}
namespace IOMUX
{
enum class Pad {
	GPIO_EMC_B1_00 = 0,
	GPIO_EMC_B1_01 = 1,
	GPIO_EMC_B1_02 = 2,
	GPIO_EMC_B1_03 = 3,
	GPIO_EMC_B1_04 = 4,
	GPIO_EMC_B1_05 = 5,
	GPIO_EMC_B1_06 = 6,
	GPIO_EMC_B1_07 = 7,
	GPIO_EMC_B1_08 = 8,
	GPIO_EMC_B1_09 = 9,
	GPIO_EMC_B1_10 = 10,
	GPIO_EMC_B1_11 = 11,
	GPIO_EMC_B1_12 = 12,
	GPIO_EMC_B1_13 = 13,
	GPIO_EMC_B1_14 = 14,
	GPIO_EMC_B1_15 = 15,
	GPIO_EMC_B1_16 = 16,
	GPIO_EMC_B1_17 = 17,
	GPIO_EMC_B1_18 = 18,
	GPIO_EMC_B1_19 = 19,
	GPIO_EMC_B1_20 = 20,
	GPIO_EMC_B1_21 = 21,
	GPIO_EMC_B1_22 = 22,
	GPIO_EMC_B1_23 = 23,
	GPIO_EMC_B1_24 = 24,
	GPIO_EMC_B1_25 = 25,
	GPIO_EMC_B1_26 = 26,
	GPIO_EMC_B1_27 = 27,
	GPIO_EMC_B1_28 = 28,
	GPIO_EMC_B1_29 = 29,
	GPIO_EMC_B1_30 = 30,
	GPIO_EMC_B1_31 = 31,
	GPIO_EMC_B1_32 = 32,
	GPIO_EMC_B1_33 = 33,
	GPIO_EMC_B1_34 = 34,
	GPIO_EMC_B1_35 = 35,
	GPIO_EMC_B1_36 = 36,
	GPIO_EMC_B1_37 = 37,
	GPIO_EMC_B1_38 = 38,
	GPIO_EMC_B1_39 = 39,
	GPIO_EMC_B1_40 = 40,
	GPIO_EMC_B1_41 = 41,
	GPIO_EMC_B2_00 = 42,
	GPIO_EMC_B2_01 = 43,
	GPIO_EMC_B2_02 = 44,
	GPIO_EMC_B2_03 = 45,
	GPIO_EMC_B2_04 = 46,
	GPIO_EMC_B2_05 = 47,
	GPIO_EMC_B2_06 = 48,
	GPIO_EMC_B2_07 = 49,
	GPIO_EMC_B2_08 = 50,
	GPIO_EMC_B2_09 = 51,
	GPIO_EMC_B2_10 = 52,
	GPIO_EMC_B2_11 = 53,
	GPIO_EMC_B2_12 = 54,
	GPIO_EMC_B2_13 = 55,
	GPIO_EMC_B2_14 = 56,
	GPIO_EMC_B2_15 = 57,
	GPIO_EMC_B2_16 = 58,
	GPIO_EMC_B2_17 = 59,
	GPIO_EMC_B2_18 = 60,
	GPIO_EMC_B2_19 = 61,
	GPIO_EMC_B2_20 = 62,
	GPIO_AD_00 = 63,
	GPIO_AD_01 = 64,
	GPIO_AD_02 = 65,
	GPIO_AD_03 = 66,
	GPIO_AD_04 = 67,
	GPIO_AD_05 = 68,
	GPIO_AD_06 = 69,
	GPIO_AD_07 = 70,
	GPIO_AD_08 = 71,
	GPIO_AD_09 = 72,
	GPIO_AD_10 = 73,
	GPIO_AD_11 = 74,
	GPIO_AD_12 = 75,
	GPIO_AD_13 = 76,
	GPIO_AD_14 = 77,
	GPIO_AD_15 = 78,
	GPIO_AD_16 = 79,
	GPIO_AD_17 = 80,
	GPIO_AD_18 = 81,
	GPIO_AD_19 = 82,
	GPIO_AD_20 = 83,
	GPIO_AD_21 = 84,
	GPIO_AD_22 = 85,
	GPIO_AD_23 = 86,
	GPIO_AD_24 = 87,
	GPIO_AD_25 = 88,
	GPIO_AD_26 = 89,
	GPIO_AD_27 = 90,
	GPIO_AD_28 = 91,
	GPIO_AD_29 = 92,
	GPIO_AD_30 = 93,
	GPIO_AD_31 = 94,
	GPIO_AD_32 = 95,
	GPIO_AD_33 = 96,
	GPIO_AD_34 = 97,
	GPIO_AD_35 = 98,
	GPIO_SD_B1_00 = 99,
	GPIO_SD_B1_01 = 100,
	GPIO_SD_B1_02 = 101,
	GPIO_SD_B1_03 = 102,
	GPIO_SD_B1_04 = 103,
	GPIO_SD_B1_05 = 104,
	GPIO_SD_B2_00 = 105,
	GPIO_SD_B2_01 = 106,
	GPIO_SD_B2_02 = 107,
	GPIO_SD_B2_03 = 108,
	GPIO_SD_B2_04 = 109,
	GPIO_SD_B2_05 = 110,
	GPIO_SD_B2_06 = 111,
	GPIO_SD_B2_07 = 112,
	GPIO_SD_B2_08 = 113,
	GPIO_SD_B2_09 = 114,
	GPIO_SD_B2_10 = 115,
	GPIO_SD_B2_11 = 116,
	GPIO_DISP_B1_00 = 117,
	GPIO_DISP_B1_01 = 118,
	GPIO_DISP_B1_02 = 119,
	GPIO_DISP_B1_03 = 120,
	GPIO_DISP_B1_04 = 121,
	GPIO_DISP_B1_05 = 122,
	GPIO_DISP_B1_06 = 123,
	GPIO_DISP_B1_07 = 124,
	GPIO_DISP_B1_08 = 125,
	GPIO_DISP_B1_09 = 126,
	GPIO_DISP_B1_10 = 127,
	GPIO_DISP_B1_11 = 128,
	GPIO_DISP_B2_00 = 129,
	GPIO_DISP_B2_01 = 130,
	GPIO_DISP_B2_02 = 131,
	GPIO_DISP_B2_03 = 132,
	GPIO_DISP_B2_04 = 133,
	GPIO_DISP_B2_05 = 134,
	GPIO_DISP_B2_06 = 135,
	GPIO_DISP_B2_07 = 136,
	GPIO_DISP_B2_08 = 137,
	GPIO_DISP_B2_09 = 138,
	GPIO_DISP_B2_10 = 139,
	GPIO_DISP_B2_11 = 140,
	GPIO_DISP_B2_12 = 141,
	GPIO_DISP_B2_13 = 142,
	GPIO_DISP_B2_14 = 143,
	GPIO_DISP_B2_15 = 144,
	WAKEUP = 145,
	PMIC_ON_REQ = 146,
	PMIC_STBY_REQ = 147,
	GPIO_SNVS_00 = 148,
	GPIO_SNVS_01 = 149,
	GPIO_SNVS_02 = 150,
	GPIO_SNVS_03 = 151,
	GPIO_SNVS_04 = 152,
	GPIO_SNVS_05 = 153,
	GPIO_SNVS_06 = 154,
	GPIO_SNVS_07 = 155,
	GPIO_SNVS_08 = 156,
	GPIO_SNVS_09 = 157,
	GPIO_LPSR_00 = 158,
	GPIO_LPSR_01 = 159,
	GPIO_LPSR_02 = 160,
	GPIO_LPSR_03 = 161,
	GPIO_LPSR_04 = 162,
	GPIO_LPSR_05 = 163,
	GPIO_LPSR_06 = 164,
	GPIO_LPSR_07 = 165,
	GPIO_LPSR_08 = 166,
	GPIO_LPSR_09 = 167,
	GPIO_LPSR_10 = 168,
	GPIO_LPSR_11 = 169,
	GPIO_LPSR_12 = 170,
	GPIO_LPSR_13 = 171,
	GPIO_LPSR_14 = 172,
	GPIO_LPSR_15 = 173
};

}

/*
 * GPIO
 */

namespace GPIO
{
enum Port {
	PortInvalid = 0,
	Port1,
	Port2,
	Port3,
	Port4,
	Port5,
	Port6,
	Port7,
	Port8,
	Port9,
	Port10,
	Port11,
	Port12,
	Port13,
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
	case GPIO::Port1: return GPIO_PORT1;

	case GPIO::Port2: return GPIO_PORT2;

	case GPIO::Port3: return GPIO_PORT3;

	case GPIO::Port4: return GPIO_PORT4;

	case GPIO::Port5: return GPIO_PORT5;

	case GPIO::Port6: return GPIO_PORT6;

	case GPIO::Port7: return GPIO_PORT7;

	case GPIO::Port8: return GPIO_PORT8;

	case GPIO::Port9: return GPIO_PORT9;

	case GPIO::Port10: return GPIO_PORT10;

	case GPIO::Port11: return GPIO_PORT11;

	case GPIO::Port12: return GPIO_PORT12;

	case GPIO::Port13: return GPIO_PORT13;

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
	LPSPI1 = 1,
	LPSPI2,
	LPSPI3,
	LPSPI4,
	LPSPI5,
	LPSPI6,
};

using CS = GPIO::GPIOPin; ///< chip-select pin
using DRDY = GPIO::GPIOPin; ///< data ready pin

struct bus_device_external_cfg_t {
	CS cs_gpio;
	DRDY drdy_gpio;
};

} // namespace SPI
