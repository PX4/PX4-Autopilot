/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file drv_gpio.h
 *
 * Generic GPIO ioctl interface.
 */

#ifndef _DRV_GPIO_H
#define _DRV_GPIO_H

#include <sys/ioctl.h>

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V1
/*
 * PX4FMU GPIO numbers.
 *
 * For shared pins, alternate function 1 selects the non-GPIO mode
 * (USART2, CAN2, etc.)
 */
# define GPIO_EXT_1		(1<<0)		/**< high-power GPIO 1 */
# define GPIO_EXT_2		(1<<1)		/**< high-power GPIO 1 */
# define GPIO_MULTI_1		(1<<2)		/**< USART2 CTS */
# define GPIO_MULTI_2		(1<<3)		/**< USART2 RTS */
# define GPIO_MULTI_3		(1<<4)		/**< USART2 TX */
# define GPIO_MULTI_4		(1<<5)		/**< USART2 RX */
# define GPIO_CAN_TX		(1<<6)		/**< CAN2 TX */
# define GPIO_CAN_RX		(1<<7)		/**< CAN2 RX */

/**
 * Device paths for things that support the GPIO ioctl protocol.
 */
# define PX4FMU_DEVICE_PATH	"/dev/px4fmu"
# define PX4IO_DEVICE_PATH	"/dev/px4io"

#endif

#ifdef CONFIG_ARCH_BOARD_PX4FMU_V2
/*
 * PX4FMUv2 GPIO numbers.
 *
 * There are no alternate functions on this board.
 */
# define GPIO_SERVO_1		(1<<0)		/**< servo 1 output */
# define GPIO_SERVO_2		(1<<1)		/**< servo 2 output */
# define GPIO_SERVO_3		(1<<2)		/**< servo 3 output */
# define GPIO_SERVO_4		(1<<3)		/**< servo 4 output */
# define GPIO_SERVO_5		(1<<4)		/**< servo 5 output */
# define GPIO_SERVO_6		(1<<5)		/**< servo 6 output */

# define GPIO_5V_PERIPH_EN	(1<<6)		/**< PA8 - !VDD_5V_PERIPH_EN */
# define GPIO_3V3_SENSORS_EN	(1<<7)		/**< PE3 - VDD_3V3_SENSORS_EN */
# define GPIO_BRICK_VALID	(1<<8)		/**< PB5 - !VDD_BRICK_VALID */
# define GPIO_SERVO_VALID	(1<<9)		/**< PB7 - !VDD_SERVO_VALID */
# define GPIO_5V_HIPOWER_OC	(1<<10)		/**< PE10 - !VDD_5V_HIPOWER_OC */
# define GPIO_5V_PERIPH_OC	(1<<11)		/**< PE10 - !VDD_5V_PERIPH_OC */

/**
 * Device paths for things that support the GPIO ioctl protocol.
 */
# define PX4FMU_DEVICE_PATH	"/dev/px4fmu"
# define PX4IO_DEVICE_PATH	"/dev/px4io"

#endif

#ifdef CONFIG_ARCH_BOARD_AEROCORE
/*
 * AeroCore GPIO numbers and configuration.
 *
 */
# define PX4FMU_DEVICE_PATH	"/dev/px4fmu"
#endif

#ifdef CONFIG_ARCH_BOARD_PX4IO_V1
/* no GPIO driver on the PX4IOv1 board */
#endif

#ifdef CONFIG_ARCH_BOARD_PX4IO_V2
/* no GPIO driver on the PX4IOv2 board */
#endif

#ifdef CONFIG_ARCH_BOARD_PX4_STM32F4DISCOVERY
/* no GPIO driver on the PX4_STM32F4DISCOVERY board */
#endif

#if !defined(CONFIG_ARCH_BOARD_PX4IO_V1) && !defined(CONFIG_ARCH_BOARD_PX4IO_V2)  && \
	!defined(CONFIG_ARCH_BOARD_PX4FMU_V1) && !defined(CONFIG_ARCH_BOARD_PX4FMU_V2) && \
	!defined(CONFIG_ARCH_BOARD_AEROCORE) && !defined(CONFIG_ARCH_BOARD_PX4_STM32F4DISCOVERY)
# error No CONFIG_ARCH_BOARD_xxxx set
#endif
/*
 * IOCTL definitions.
 *
 * For all ioctls, the (arg) argument is a bitmask of GPIOs to be affected
 * by the operation, with the LSB being the lowest-numbered GPIO.
 *
 * Note that there may be board-specific relationships between GPIOs;
 * applications using GPIOs should be aware of this.
 */
#define _GPIOCBASE	0x2700
#define GPIOC(_x)	_IOC(_GPIOCBASE, _x)

/** reset all board GPIOs to their default state */
#define GPIO_RESET	GPIOC(0)

/** configure the board GPIOs in (arg) as outputs */
#define GPIO_SET_OUTPUT	GPIOC(1)

/** configure the board GPIOs in (arg) as inputs */
#define GPIO_SET_INPUT	GPIOC(2)

/** configure the board GPIOs in (arg) for the first alternate function (if supported) */
#define GPIO_SET_ALT_1	GPIOC(3)

/** configure the board GPIO (arg) for the second alternate function (if supported) */
#define GPIO_SET_ALT_2	GPIOC(4)

/** configure the board GPIO (arg) for the third alternate function (if supported) */
#define GPIO_SET_ALT_3	GPIOC(5)

/** configure the board GPIO (arg) for the fourth alternate function (if supported) */
#define GPIO_SET_ALT_4	GPIOC(6)

/** set the GPIOs in (arg) */
#define GPIO_SET	GPIOC(10)

/** clear the GPIOs in (arg) */
#define GPIO_CLEAR	GPIOC(11)

/** read all the GPIOs and return their values in *(uint32_t *)arg */
#define GPIO_GET	GPIOC(12)

#define GPIO_SENSOR_RAIL_RESET	GPIOC(13)

#define GPIO_PERIPHERAL_RAIL_RESET	GPIOC(14)

#endif /* _DRV_GPIO_H */
