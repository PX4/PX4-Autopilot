/****************************************************************************
 *
 *   Copyright (C) 2014 PX4 Development Team. All rights reserved.
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
 * @file vehicle_force_setpoint.h
 * @author Thomas Gubler <thomasgubler@gmail.com>
 * Definition of force (NED) setpoint uORB topic. Typically this can be used
 * by a position control app together with an attitude control app.
 */

#ifndef TOPIC_VEHICLE_FORCE_SETPOINT_H_
#define TOPIC_VEHICLE_FORCE_SETPOINT_H_

#include "../uORB.h"

/**
 * @addtogroup topics
 * @{
 */

struct vehicle_force_setpoint_s {
	float x;		/**< in N NED			  		*/
	float y;		/**< in N NED			  		*/
	float z;		/**< in N NED			  		*/
	float yaw;		/**< right-hand rotation around downward axis (rad, equivalent to Tait-Bryan yaw) */
}; /**< Desired force in NED frame */

/**
 * @}
 */

/* register this as object request broker structure */
ORB_DECLARE(vehicle_force_setpoint);

#endif
