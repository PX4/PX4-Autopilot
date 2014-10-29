/****************************************************************************
 *
 *   Copyright (c) 2012-2014 PX4 Development Team. All rights reserved.
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
 * @file vehicle_global_position.h
 * Definition of the global fused WGS84 position uORB topic.
 *
 * @author Thomas Gubler <thomasgubler@student.ethz.ch>
 */

#ifndef TECS_STATUS_T_H_
#define TECS_STATUS_T_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

typedef enum {
	TECS_MODE_NORMAL = 0,
	TECS_MODE_UNDERSPEED,
	TECS_MODE_TAKEOFF,
	TECS_MODE_LAND,
	TECS_MODE_LAND_THROTTLELIM,
	TECS_MODE_BAD_DESCENT,
	TECS_MODE_CLIMBOUT
} tecs_mode;

 /**
 * Internal values of the (m)TECS fixed wing speed alnd altitude control system
 */
struct tecs_status_s {
	uint64_t timestamp;		/**< timestamp, in microseconds since system start */

	float altitudeSp;
	float altitude_filtered;
	float flightPathAngleSp;
	float flightPathAngle;
	float flightPathAngleFiltered;
	float airspeedSp;
	float airspeed_filtered;
	float airspeedDerivativeSp;
	float airspeedDerivative;

	float totalEnergyRateSp;
	float totalEnergyRate;
	float energyDistributionRateSp;
	float energyDistributionRate;

	tecs_mode mode;
};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(tecs_status);

#endif
