/****************************************************************************
 *
 *   Copyright (C) 2013 PX4 Development Team. All rights reserved.
 *   Author: @author Marco Bauer <marco@wtns.de>
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
 * @file esc_status.h
 * Definition of the esc_status uORB topic.
 *
 * Published the state machine and the system status bitfields
 * (see SYS_STATUS mavlink message), published only by commander app.
 *
 * All apps should write to subsystem_info:
 *
 *  (any app) --> subsystem_info (published) --> (commander app state machine)  --> esc_status --> (mavlink app)
 */

#ifndef ESC_STATUS_H_
#define ESC_STATUS_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * The number of ESCs supported.
 * Current (Q2/2013) we support 8 ESCs,
 */
#define CONNECTED_ESC_MAX   8

enum ESC_VENDOR {
	ESC_VENDOR_GENERIC = 0,					/**< generic ESC */
	ESC_VENDOR_MIKROKOPTER,					/**< Mikrokopter */
	ESC_VENDOR_GRAUPNER_HOTT				/**< Graupner HoTT ESC */
};

enum ESC_CONNECTION_TYPE {
	ESC_CONNECTION_TYPE_PPM = 0,			/**< Traditional PPM ESC */
	ESC_CONNECTION_TYPE_SERIAL,			/**< Serial Bus connected ESC */
	ESC_CONNECTION_TYPE_ONESHOOT,			/**< One Shoot PPM */
	ESC_CONNECTION_TYPE_I2C,				/**< I2C */
	ESC_CONNECTION_TYPE_CAN					/**< CAN-Bus */
};

/**
 * @addtogroup topics
 * @{
 */

/**
 * Electronic speed controller status.
 * Unsupported float fields should be assigned NaN.
 */
struct esc_status_s {
	/* use of a counter and timestamp recommended (but not necessary) */

	uint16_t counter;   		/**< incremented by the writing thread everytime new data is stored */
	uint64_t timestamp; 		/**< in microseconds since system start, is set whenever the writing thread stores new data */

	uint8_t esc_count;			/**< number of connected ESCs */
	enum ESC_CONNECTION_TYPE esc_connectiontype;	/**< how ESCs connected to the system */

	struct {
		enum ESC_VENDOR esc_vendor;		/**< Vendor of current ESC */
		uint32_t esc_errorcount;		/**< Number of reported errors by ESC - if supported */
                int32_t esc_rpm;                        /**< Motor RPM, negative for reverse rotation [RPM] - if supported */
		float esc_voltage;			/**< Voltage measured from current ESC [V] - if supported */
		float esc_current;			/**< Current measured from current ESC [A] - if supported */
		float esc_temperature;			/**< Temperature measured from current ESC [degC] - if supported */
		float esc_setpoint;			/**< setpoint of current ESC */
		uint16_t esc_setpoint_raw;		/**< setpoint of current ESC (Value sent to ESC) */
		uint16_t esc_address;			/**< Address of current ESC (in most cases 1-8 / must be set by driver) */
		uint16_t esc_version;			/**< Version of current ESC - if supported */
		uint16_t esc_state;			/**< State of ESC - depend on Vendor */
	} esc[CONNECTED_ESC_MAX];

};

/**
 * @}
 */

/* register this as object request broker structure */
//ORB_DECLARE(esc_status);
ORB_DECLARE_OPTIONAL(esc_status);

#endif
