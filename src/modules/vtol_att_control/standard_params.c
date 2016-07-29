/****************************************************************************
 *
 *   Copyright (c) 2015 PX4 Development Team. All rights reserved.
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
 * @file standard_params.c
 * Parameters for the standard VTOL controller.
 *
 * @author Simon Wilks	<simon@uaventure.com>
 * @author Roman Bapst	<roman@px4.io>
 */

/**
 * Target throttle value for pusher/puller motor during the transition to fw mode
 *
 * @min 0.0
 * @max 1.0
 * @increment 0.01
 * @decimal 3
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_TRANS_THR, 0.6f);

/**
 * Maximum allowed down-pitch the controller is able to demand. This prevents large, negative
 * lift values being created when facing strong winds. The vehicle will use the pusher motor
 * to accelerate forward if necessary.
 *
 * @min 0.0
 * @max 45.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_DWN_PITCH_MAX, 5.0f);

/**
 * Fixed wing thrust scale for hover forward flight.
 *
 * Scale applied to fixed wing thrust being used as source for forward acceleration in multirotor mode.
 * This technique can be used to avoid the plane having to pitch down a lot in order to move forward.
 * Setting this value to 0 (default) will disable this strategy.
 * @min 0.0
 * @max 2.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FWD_THRUST_SC, 0.0f);


/**
 * QuadChute
 *
 * Minimum altitude for fixed wing flight, when in fixed wing the altitude drops below this altitude
 * the vehicle will transition back to MC mode and enter failsafe RTL
 * @min 0.0
 * @max 200.0
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FW_MIN_ALT, 0.0f);
