/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
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
 * AS IS AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
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
 * @file ams_params.c
 *
 * Parameters for AMS
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 */

/*
 * AMS parameters, accessible via MAVLink
 */

/**
 * AMS velocity
 *
 * Downward target velocity during emergency descend phase
 *
 * @unit m/s
 * @min 1
 * @max 20
 * @decimal 1
 * @increment 0.1
 * @group Auto Maneuver System
 */
PARAM_DEFINE_FLOAT(AMS_DESCEND_VEL, 5);

/**
 * AMS loiter altitude
 *
 * Descend to this altitude. Above ground if possible, relative to home position
 * if ground distance is not available.
 * Loiter at this altitude if AMS_TYPE = Loiter
 * Land (i.e. slowly descend) from this altitude if AMS_TYPE = Land.
 * Go to rally point at this altitude if AMS_TYPE = Rally
 *
 * @unit m
 * @min 2
 * @max 200
 * @decimal 1
 * @increment 0.5
 * @group Auto Maneuver System
 */
PARAM_DEFINE_FLOAT(AMS_DESCEND_ALT, 50);

/**
 * Return type
 *
 * Fly straight to the home location or planned mission landing and land there or
 * use the planned mission to get to those points.
 *
 * @value 0 Descend and loiter
 * @value 1 Descend and land
 * @value 2 Descend and go to closest rally point
 * @group Auto Maneuver System
 */
PARAM_DEFINE_INT32(AMS_TYPE, 0);
