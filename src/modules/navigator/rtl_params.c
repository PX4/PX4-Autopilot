/****************************************************************************
 *
 *   Copyright (c) 2014-2016 PX4 Development Team. All rights reserved.
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
 * @file rtl_params.c
 *
 * Parameters for RTL
 *
 * @author Julian Oes <julian@oes.ch>
 */

/*
 * RTL parameters, accessible via MAVLink
 */

/**
 * RTL altitude
 *
 * Altitude to fly back in RTL in meters
 *
 * @unit m
 * @min 0
 * @max 150
 * @decimal 1
 * @increment 0.5
 * @group Return Mode
 */
PARAM_DEFINE_FLOAT(RTL_RETURN_ALT, 60);


/**
 * Return mode loiter altitude
 *
 * Stay at this altitude above home position after RTL descending.
 * Land (i.e. slowly descend) from this altitude if autolanding allowed.
 *
 * @unit m
 * @min 2
 * @max 100
 * @decimal 1
 * @increment 0.5
 * @group Return Mode
 */
PARAM_DEFINE_FLOAT(RTL_DESCEND_ALT, 30);

/**
 * Return mode delay
 *
 * Delay after descend before landing in Return mode.
 * If set to -1 the system will not land but loiter at RTL_DESCEND_ALT.
 *
 * @unit s
 * @min -1
 * @max 300
 * @decimal 1
 * @increment 0.5
 * @group Return Mode
 */
PARAM_DEFINE_FLOAT(RTL_LAND_DELAY, -1.0f);

/**
 * Minimum distance to trigger rising to a safe altitude
 *
 * If the system is horizontally closer than this distance to home
 * it will land straight on home instead of raising to the return
 * altitude first.
 *
 * @unit m
 * @min 0.5
 * @max 20
 * @decimal 1
 * @increment 0.5
 * @group Return Mode
 */
PARAM_DEFINE_FLOAT(RTL_MIN_DIST, 5.0f);

/**
 * Return type
 *
 * Fly straight to the home location or planned mission landing and land there or
 * use the planned mission to get to those points.
 *
 * @value 0 Return home via direct path
 * @value 1 Return to a planned mission landing, if available, via direct path, else return to home via direct path
 * @value 2 Return to a planned mission landing, if available, using the mission path, else return to home via the reverse mission path
 * @group Return Mode
 */
PARAM_DEFINE_INT32(RTL_TYPE, 0);
