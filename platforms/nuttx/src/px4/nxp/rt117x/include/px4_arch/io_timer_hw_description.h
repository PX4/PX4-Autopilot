/****************************************************************************
 *
 *   Copyright (C) 2019 PX4 Development Team. All rights reserved.
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


#include <px4_arch/io_timer.h>
#include <px4_arch/hw_description.h>
#include <px4_platform_common/constexpr_util.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform/io_timer_init.h>

#include <hardware/imxrt_tmr.h>
#include <hardware/imxrt_flexpwm.h>
#include <imxrt_gpio.h>
#include <imxrt_iomuxc.h>
#include <hardware/imxrt_pinmux.h>

#include <board_config.h>
#if !defined(CONFIG_ARCH_CHIP_MIMXRT1176DVMAA)
# error "This code has only been validated with IMXRT1176. Make sure it is correct before using it on another board."
#endif


static inline constexpr timer_io_channels_t initIOTimerChannel(const io_timers_t io_timers_conf[MAX_IO_TIMERS],
		PWM::FlexPWMConfig pwm_config, IOMUX::Pad pad)
{
	timer_io_channels_t ret{};
	PWM::FlexPWM pwm {};

	// FlexPWM Muxing Options
	switch (pwm_config.module) {
	case PWM::PWM1_PWM_A:
		pwm = PWM::FlexPWM1;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_23) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_23_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN23;

			} else if (pad == IOMUX::Pad::GPIO_AD_00) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_00_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN31;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_25) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_25_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN25;

			} else if (pad == IOMUX::Pad::GPIO_AD_02) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_02_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN1;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_27) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_27_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN27;

			} else if (pad == IOMUX::Pad::GPIO_AD_04) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_04_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN3;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_38) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_38_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN6;

			}

			break;
		}

		break;

	case PWM::PWM1_PWM_B:
		pwm = PWM::FlexPWM1;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_24) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_24_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN24;

			} else if (pad == IOMUX::Pad::GPIO_AD_01) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_01_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN0;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_26) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_26_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN26;

			} else if (pad == IOMUX::Pad::GPIO_AD_03) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_03_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN2;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_28) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_28_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN28;

			} else if (pad == IOMUX::Pad::GPIO_AD_05) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_05_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN4;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_39) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_39_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN7;

			}

			break;
		}

		break;

	case PWM::PWM1_PWM_X:
		pwm = PWM::FlexPWM1;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_AD_06) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_06_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN5;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_AD_07) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_07_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN6;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_AD_08) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_08_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN7;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_AD_09) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_09_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN8;
			}

			break;
		}

		break;


	case PWM::PWM2_PWM_A:
		pwm = PWM::FlexPWM2;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_06) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_06_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN6;

			} else if (pad == IOMUX::Pad::GPIO_AD_24) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_24_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN23;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_08) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_08_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN8;

			} else if (pad == IOMUX::Pad::GPIO_AD_26) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_26_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN25;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_10) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_10_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN10;

			} else if (pad == IOMUX::Pad::GPIO_AD_28) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_28_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN27;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_19) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_19_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN19;

			}

			break;
		}

		break;

	case PWM::PWM2_PWM_B:
		pwm = PWM::FlexPWM2;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_07) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_07_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN7;

			} else if (pad == IOMUX::Pad::GPIO_AD_25) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_25_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN24;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_09) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_09_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN9;

			} else if (pad == IOMUX::Pad::GPIO_AD_27) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_27_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN26;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_11) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_11_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN11;

			} else if (pad == IOMUX::Pad::GPIO_AD_29) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT4 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_29_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN28;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_20) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_20_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN20;

			}

			break;
		}

		break;

	case PWM::PWM2_PWM_X:
		pwm = PWM::FlexPWM2;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_AD_10) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_10_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN9;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_AD_11) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_11_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN10;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_AD_12) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_12_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN11;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_AD_13) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_13_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN12;
			}

			break;
		}

		break;


	case PWM::PWM3_PWM_A:
		pwm = PWM::FlexPWM3;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_29) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_29_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN29;

			} else if (pad == IOMUX::Pad::GPIO_EMC_B2_00) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B2_00_INDEX);
				ret.gpio_portpin = GPIO_PORT2 | GPIO_PIN10;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_31) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_31_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN31;

			} else if (pad == IOMUX::Pad::GPIO_EMC_B2_02) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B2_02_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN12;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_33) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_33_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN1;

			} else if (pad == IOMUX::Pad::GPIO_EMC_B2_04) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B2_04_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN14;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_21) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_21_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN21;

			} else if (pad == IOMUX::Pad::GPIO_EMC_B2_06) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B2_06_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN16;
			}

			break;
		}

		break;

	case PWM::PWM3_PWM_B:
		pwm = PWM::FlexPWM3;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_30) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_30_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN30;

			} else if (pad == IOMUX::Pad::GPIO_EMC_B2_01) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B2_01_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN11;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_32) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_32_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN0;

			} else if (pad == IOMUX::Pad::GPIO_EMC_B2_03) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B2_03_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN13;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_34) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_34_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN2;

			} else if (pad == IOMUX::Pad::GPIO_EMC_B2_05) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B2_05_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN15;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_22) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_22_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN22;

			} else if (pad == IOMUX::Pad::GPIO_EMC_B2_07) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B2_07_INDEX);
				ret.gpio_portpin = GPIO_PORT8 | GPIO_PIN17;
			}

			break;
		}

		break;


	case PWM::PWM3_PWM_X:
		pwm = PWM::FlexPWM3;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_AD_14) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_14_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN13;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_AD_15) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_15_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN14;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_AD_16) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_16_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN15;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_AD_17) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_17_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN16;
			}

			break;
		}

		break;

	case PWM::PWM4_PWM_A:
		pwm = PWM::FlexPWM4;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_00) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_00_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN0;

			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_02) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_02_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN2;

			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_04) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_04_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN4;

			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_17) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_17_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN17;

			}

			break;
		}

		break;

	case PWM::PWM4_PWM_B:
		pwm = PWM::FlexPWM4;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_01) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_01_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN1;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_03) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_03_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN3;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_05) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_05_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN5;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_EMC_B1_18) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT1 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_EMC_B1_18_INDEX);
				ret.gpio_portpin = GPIO_PORT1 | GPIO_PIN18;
			}

			break;
		}

		break;


	case PWM::PWM4_PWM_X:
		pwm = PWM::FlexPWM4;

		switch (pwm_config.submodule) {
		case PWM::Submodule0:
			if (pad == IOMUX::Pad::GPIO_AD_18) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_18_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN17;
			}

			break;

		case PWM::Submodule1:
			if (pad == IOMUX::Pad::GPIO_AD_19) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_19_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN18;
			}

			break;

		case PWM::Submodule2:
			if (pad == IOMUX::Pad::GPIO_AD_20) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_20_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN19;
			}

			break;

		case PWM::Submodule3:
			if (pad == IOMUX::Pad::GPIO_AD_21) {
				ret.gpio_out = GPIO_PERIPH | GPIO_ALT11 | GPIO_PADMUX(IMXRT_PADMUX_GPIO_AD_21_INDEX);
				ret.gpio_portpin = GPIO_PORT9 | GPIO_PIN20;
			}

			break;
		}

		break;
	}

	constexpr_assert(ret.gpio_out != 0, "Invalid PWM/Pad config");
	ret.gpio_out |= IOMUX_CMOS_OUTPUT | IOMUX_PULL_NONE | IOMUX_DRIVE_HIGHSTRENGTH | IOMUX_SLEW_FAST;

	switch (pwm_config.module) {
	case PWM::PWM1_PWM_A:
	case PWM::PWM2_PWM_A:
	case PWM::PWM3_PWM_A:
	case PWM::PWM4_PWM_A:
		ret.val_offset = PWMA_VAL;
		break;

	case PWM::PWM1_PWM_B:
	case PWM::PWM2_PWM_B:
	case PWM::PWM3_PWM_B:
	case PWM::PWM4_PWM_B:
		ret.val_offset = PWMB_VAL;
		break;

	case PWM::PWM1_PWM_X:
	case PWM::PWM2_PWM_X:
	case PWM::PWM3_PWM_X:
	case PWM::PWM4_PWM_X:
		ret.val_offset = PWMX_VAL;
		break;

	default:
		constexpr_assert(false, "not implemented");
	}

	switch (pwm_config.submodule) {
	case PWM::Submodule0:
		ret.sub_module = SM0;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM0);
		break;

	case PWM::Submodule1:
		ret.sub_module = SM1;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM1);
		break;

	case PWM::Submodule2:
		ret.sub_module = SM2;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM2);
		break;

	case PWM::Submodule3:
		ret.sub_module = SM3;
		ret.sub_module_bits = MCTRL_LDOK(1 << SM3);
		break;
	}

	ret.gpio_in = 0; // TODO (not used yet)

	// find timer index
	ret.timer_index = 0xff;
	const uint32_t timer_base = getFlexPWMBaseRegister(pwm);

	for (int i = 0; i < MAX_IO_TIMERS; ++i) {
		if (io_timers_conf[i].base == timer_base && io_timers_conf[i].submodle == ret.sub_module) {
			ret.timer_index = i;
			break;
		}
	}

	constexpr_assert(ret.timer_index != 0xff, "Timer not found");

	return ret;
}

static inline constexpr io_timers_t initIOPWM(PWM::FlexPWM pwm, PWM::FlexPWMSubmodule sub)
{
	io_timers_t ret{};

	ret.base = getFlexPWMBaseRegister(pwm);
	ret.submodle = sub;
	return ret;
}
