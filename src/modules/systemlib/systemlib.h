/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
 *   Author: Lorenz Meier <lm@inf.ethz.ch>
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
 * @file systemlib.h
 * Definition of commonly used low-level system-call like functions.
 */

#ifndef SYSTEMLIB_H_
#define SYSTEMLIB_H_
#include <float.h>
#include <stdint.h>
#include <sched.h>

__BEGIN_DECLS

/** Reboots the board */
__EXPORT void systemreset(bool to_bootloader) noreturn_function;

/** Sends SIGUSR1 to all processes */
__EXPORT void killall(void);

/** Default scheduler type */
#if CONFIG_RR_INTERVAL > 0
# define SCHED_DEFAULT	SCHED_RR
#else
# define SCHED_DEFAULT	SCHED_FIFO
#endif

/** Starts a task and performs any specific accounting, scheduler setup, etc. */
__EXPORT int task_spawn_cmd(const char *name,
			int priority,
			int scheduler,
			int stack_size,
			main_t entry,
			const char *argv[]);

enum MULT_PORTS {
	MULT_0_US2_RXTX = 0,
	MULT_1_US2_FLOW,
	MULT_2_GPIO_12,
	MULT_COUNT
};

/* Check max multi port count */
#if (MULT_COUNT > 33)
#error "MULT_COUNT HAS TO BE LESS THAN OR EQUAL 33"
#endif

/* FMU board info, to be stored in the first 64 bytes of the FMU EEPROM */
#pragma pack(push,1)
struct fmu_board_info_s {
	char header[3];				/**< {'P', 'X', '4'} */
	char board_name[20];			/**< Human readable board name, \0 terminated */
	uint8_t board_id;			/**< board ID, constantly increasing number per board */
	uint8_t board_version;			/**< board version, major * 10 + minor: v1.7 = 17 */
	uint8_t multi_port_config[MULT_COUNT];	/**< Configuration of multi ports 1-3		  */
	uint8_t reserved[33 - MULT_COUNT];	/**< Reserved space for more multi ports	  */
	uint16_t production_year;
	uint8_t production_month;
	uint8_t production_day;
	uint8_t production_fab;
	uint8_t production_tester;
}; /**< stores autopilot board information meta data from EEPROM */
#pragma pack(pop)

/* Carrier board info, to be stored in the 128 byte board info EEPROM */
#pragma pack(push,1)
struct carrier_board_info_s {
	char header[3];				/**< {'P', 'X', '4'} */
	char board_name[20];			/**< Human readable board name, \0 terminated */
	uint8_t board_id;			/**< board ID, constantly increasing number per board */
	uint8_t board_version;			/**< board version, major * 10 + minor: v1.7 = 17 */
	uint8_t multi_port_config[MULT_COUNT];	/**< Configuration of multi ports 1-3		  */
	uint8_t reserved[33 - MULT_COUNT];	/**< Reserved space for more multi ports	  */
	uint16_t production_year;
	uint8_t production_month;
	uint8_t production_day;
	uint8_t production_fab;
	uint8_t production_tester;
	char board_custom_data[64];
}; /**< stores carrier board information meta data from EEPROM */
#pragma pack(pop)

struct __multiport_info {
	const char *port_names[MULT_COUNT];
};
__EXPORT extern const struct __multiport_info multiport_info;

__END_DECLS

#endif /* SYSTEMLIB_H_ */
