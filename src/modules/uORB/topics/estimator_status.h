/****************************************************************************
 *
 *   Copyright (c) 2014 PX4 Development Team. All rights reserved.
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
 * @file estimator_status.h
 * Definition of the estimator_status_report uORB topic.
 *
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */

#ifndef ESTIMATOR_STATUS_H_
#define ESTIMATOR_STATUS_H_

#include <stdint.h>
#include <stdbool.h>
#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

/**
 * Estimator status report.
 *
 * This is a generic status report struct which allows any of the onboard estimators
 * to write the internal state to the system log.
 *
 */
struct estimator_status_report {

	/* NOTE: Ordering of fields optimized to align to 32 bit / 4 bytes - change with consideration only   */

	uint64_t timestamp;			/**< Timestamp in microseconds since boot */
	float states[32];			/**< Internal filter states */
	float n_states;				/**< Number of states effectively used */
	uint8_t nan_flags;			/**< Bitmask to indicate NaN states */
	uint8_t health_flags;			/**< Bitmask to indicate sensor health states (vel, pos, hgt) */
	uint8_t timeout_flags;			/**< Bitmask to indicate timeout flags (vel, pos, hgt) */

};

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(estimator_status);

#endif
