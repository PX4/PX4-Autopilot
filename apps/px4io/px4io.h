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
 * @file px4io.h
 *
 * General defines and structures for the PX4IO module firmware.
 */

#include <nuttx/config.h>

#include <stdbool.h>
#include <stdint.h>

#include <drivers/boards/px4io/px4io_internal.h>

#include "protocol.h"

/*
 * Constants and limits.
 */
#define MAX_CONTROL_CHANNELS	12
#define IO_SERVO_COUNT		8

/*
 * Debug logging
 */

#ifdef DEBUG
# include <debug.h>
# define debug(fmt, args...)	lowsyslog(fmt "\n", ##args)
#else
# define debug(fmt, args...)	do {} while(0)
#endif

/*
 * System state structure.
 */
struct sys_state_s {

	bool		armed;			/* IO armed */
	bool		arm_ok;			/* FMU says OK to arm */
	uint16_t	servo_rate;		/* output rate of servos in Hz */

	/**
	 * Remote control input(s) channel mappings
	 */
	uint8_t		rc_map[4];

	/**
	 * Remote control channel attributes
	 */
	uint16_t	rc_min[4];
	uint16_t	rc_trim[4];
	uint16_t	rc_max[4];
	int16_t		rc_rev[4];
	uint16_t	rc_dz[4];

	/**
	 * Data from the remote control input(s)
	 */
	unsigned	rc_channels;
	uint16_t	rc_channel_data[PX4IO_INPUT_CHANNELS];
	uint64_t	rc_channels_timestamp;

	/**
	 * Control signals from FMU.
	 */
	uint16_t	fmu_channel_data[PX4IO_CONTROL_CHANNELS];

	/**
	 * Mixed servo outputs
	 */
	uint16_t	servos[IO_SERVO_COUNT];

	/**
	 * Relay controls
	 */
	bool		relays[PX4IO_RELAY_CHANNELS];

	/**
	 * If true, we are using the FMU controls, else RC input if available.
	 */
	bool		mixer_manual_override;

	/**
	 * If true, FMU input is available.
	 */
	bool		mixer_fmu_available;

	/**
	 * If true, state that should be reported to FMU has been updated.
	 */
	bool		fmu_report_due;

	/**
	 * Last FMU receive time, in microseconds since system boot
	 */
	uint64_t	fmu_data_received_time;

	/**
	 * Current serial interface mode, per the serial_rx_mode parameter
	 * in the config packet.
	 */
	uint8_t		serial_rx_mode;

	/**
	 * If true, the RC signal has been lost for more than a timeout interval
	 */
	bool		rc_lost;

	/**
	 * If true, the connection to FMU has been lost for more than a timeout interval
	 */
	bool		fmu_lost;

	/**
	 * If true, FMU is ready for autonomous position control. Only used for LED indication
	 */
	bool vector_flight_mode_ok;

	/**
	 * If true, IO performs an on-board manual override with the RC channel values
	 */
	bool manual_override_ok;

	/*
	 * Measured battery voltage in mV
	 */
	uint16_t	battery_mv;

	/*
	 * ADC IN5 measurement
	 */
	uint16_t	adc_in5;

	/*
	 * Power supply overcurrent status bits.
	 */
	uint8_t		overcurrent;

};

extern struct sys_state_s system_state;

#if 0
/*
 * Software countdown timers.
 *
 * Each timer counts down to zero at one tick per ms.
 */
#define TIMER_BLINK_AMBER	0
#define TIMER_BLINK_BLUE	1
#define TIMER_STATUS_PRINT	2
#define TIMER_SANITY		7
#define TIMER_NUM_TIMERS	8
extern volatile int	timers[TIMER_NUM_TIMERS];
#endif

/*
 * GPIO handling.
 */
#define LED_BLUE(_s)		stm32_gpiowrite(GPIO_LED1, !(_s))
#define LED_AMBER(_s)		stm32_gpiowrite(GPIO_LED2, !(_s))
#define LED_SAFETY(_s)		stm32_gpiowrite(GPIO_LED3, !(_s))

#define POWER_SERVO(_s)		stm32_gpiowrite(GPIO_SERVO_PWR_EN, (_s))
#define POWER_ACC1(_s)		stm32_gpiowrite(GPIO_ACC1_PWR_EN, (_s))
#define POWER_ACC2(_s)		stm32_gpiowrite(GPIO_ACC2_PWR_EN, (_s))
#define POWER_RELAY1(_s)	stm32_gpiowrite(GPIO_RELAY1_EN, (_s))
#define POWER_RELAY2(_s)	stm32_gpiowrite(GPIO_RELAY2_EN, (_s))

#define OVERCURRENT_ACC		(!stm32_gpioread(GPIO_ACC_OC_DETECT))
#define OVERCURRENT_SERVO	(!stm32_gpioread(GPIO_SERVO_OC_DETECT))
#define BUTTON_SAFETY		stm32_gpioread(GPIO_BTN_SAFETY)

#define ADC_VBATT		4
#define ADC_IN5			5

/*
 * Mixer
 */
extern void	mixer_tick(void);
extern void	mixer_handle_text(const void *buffer, size_t length);

/*
 * Safety switch/LED.
 */
extern void	safety_init(void);

/*
 * FMU communications
 */
extern void	comms_main(void) __attribute__((noreturn));

/*
 * Sensors/misc inputs
 */
extern int	adc_init(void);
extern uint16_t	adc_measure(unsigned channel);

/*
 * R/C receiver handling.
 */
extern void	controls_main(void);
extern int	dsm_init(const char *device);
extern bool	dsm_input(void);
extern int	sbus_init(const char *device);
extern bool	sbus_input(void);

/*
 * Assertion codes
 */
#define A_GPIO_OPEN_FAIL		100
#define A_SERVO_OPEN_FAIL		101
#define A_INPUTQ_OPEN_FAIL		102
