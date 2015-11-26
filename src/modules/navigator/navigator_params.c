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
 * @file navigator_params.c
 *
 * Parameters for navigator in general
 *
 * @author Julian Oes <julian@px4.io>
 * @author Thomas Gubler <thomas@px4.io>
 */

/**
 * Loiter radius (FW only)
 *
 * Default value of loiter radius for missions, loiter, RTL, etc. (fixedwing only).
 *
 * @unit m
 * @min 25
 * @max 1000
 * @decimal 1
 * @increment 0.5
 * @group Mission
 */
PARAM_DEFINE_FLOAT(NAV_LOITER_RAD, 50.0f);

/**
 * Acceptance Radius
 *
 * Default acceptance radius, overridden by acceptance radius of waypoint if set.
 *
 * @unit m
 * @min 0.05
 * @max 200.0
 * @decimal 1
 * @increment 0.5
 * @group Mission
 */
PARAM_DEFINE_FLOAT(NAV_ACC_RAD, 10.0f);

/**
 * Set data link loss failsafe mode
 *
 * The data link loss failsafe will only be entered after a timeout,
 * set by COM_DL_LOSS_T in seconds. Once the timeout occurs the selected
 * action will be executed. Setting this parameter to 4 will enable CASA
 * Outback Challenge rules, which are only recommended to participants
 * of that competition.
 *
 * @value 0 Disabled
 * @value 1 Loiter
 * @value 2 Return to Land
 * @value 3 Land at current position
 *
 * @group Mission
 */
PARAM_DEFINE_INT32(NAV_DLL_ACT, 0);

/**
 * Set RC loss failsafe mode
 *
 * The RC loss failsafe will only be entered after a timeout,
 * set by COM_RC_LOSS_T in seconds. If RC input checks have been disabled
 * by setting the COM_RC_IN_MODE param it will not be triggered.
 * Setting this parameter to 4 will enable CASA Outback Challenge rules,
 * which are only recommended to participants of that competition.
 *
 * @value 0 Disabled
 * @value 1 Loiter
 * @value 2 Return to Land
 * @value 3 Land at current position
 *
 * @group Mission
 */
PARAM_DEFINE_INT32(NAV_RCL_ACT, 0);

/**
 * Set offboard loss failsafe mode
 *
 * The offboard loss failsafe will only be entered after a timeout,
 * set by COM_OF_LOSS_T in seconds.
 *
 * @value 0 Land at current position
 * @value 1 Loiter
 * @value 2 Return to Land
 *
 * @group Mission
 */
PARAM_DEFINE_INT32(NAV_OBL_ACT, 0);

/**
 * Set offboard loss failsafe mode when RC is available
 *
 * The offboard loss failsafe will only be entered after a timeout,
 * set by COM_OF_LOSS_T in seconds.
 *
 * @value 0 Position control
 * @value 1 Altitude control
 * @value 2 Manual
 * @value 3 Return to Land
 * @value 4 Land at current position
 *
 * @group Mission
 */
PARAM_DEFINE_INT32(NAV_OBL_RC_ACT, 0);

/**
 * Airfield home Lat
 *
 * Latitude of airfield home waypoint
 *
 * @unit deg * 1e7
 * @min -900000000
 * @max 900000000
 * @group Data Link Loss
 */
PARAM_DEFINE_INT32(NAV_AH_LAT, -265847810);

/**
 * Airfield home Lon
 *
 * Longitude of airfield home waypoint
 *
 * @unit deg * 1e7
 * @min -1800000000
 * @max 1800000000
 * @group Data Link Loss
 */
PARAM_DEFINE_INT32(NAV_AH_LON, 1518423250);

/**
 * Airfield home alt
 *
 * Altitude of airfield home waypoint
 *
 * @unit m
 * @min -50
 * @decimal 1
 * @increment 0.5
 * @group Data Link Loss
 */
PARAM_DEFINE_FLOAT(NAV_AH_ALT, 600.0f);
