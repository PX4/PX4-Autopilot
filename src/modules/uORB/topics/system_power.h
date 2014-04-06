/****************************************************************************
 *
 *   Copyright (C) 2012-2013 PX4 Development Team. All rights reserved.
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
 * @file system_power.h
 *
 * Definition of the system_power voltage and power status uORB topic.
 */

#ifndef SYSTEM_POWER_H_
#define SYSTEM_POWER_H_

#include "../uORB.h"
#include <stdint.h>

/**
 * @addtogroup topics
 * @{
 */

/**
 * voltage and power supply status
 */
struct system_power_s {
	uint64_t	timestamp;		/**< microseconds since system boot */
	float   	voltage5V_v;		/**< peripheral 5V rail voltage */
	uint8_t         usb_connected:1;        /**< USB is connected when 1 */
	uint8_t         brick_valid:1;          /**< brick power is good when 1 */
	uint8_t         servo_valid:1;          /**< servo power is good when 1 */
	uint8_t		periph_5V_OC:1;         /**< peripheral overcurrent when 1 */
	uint8_t		hipower_5V_OC:1;        /**< hi power peripheral overcurrent when 1 */
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(system_power);

#endif
