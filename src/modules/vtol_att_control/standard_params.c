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
 * Use fixed-wing actuation in hover to accelerate forward
 *
 * Prevents downforce from pitching the body down when facing wind.
 * Uses puller/pusher (standard VTOL), or forward-tilt (tiltrotor VTOL) to accelerate forward instead.
 * Only active if demanded pitch  is below VT_PITCH_MIN.
 * Use VT_FWD_THRUST_SC to tune it.
 * Descend mode is treated as Landing too.
 *
 * Only active (if enabled) in height-rate controlled modes.
 *
 * @value 0 Disabled
 * @value 1 Enabled (except LANDING)
 * @value 2 Enabled if above MPC_LAND_ALT1
 * @value 3 Enabled if above MPC_LAND_ALT2
 * @value 4 Enabled constantly
 * @value 5 Enabled if above MPC_LAND_ALT1 (except LANDING)
 * @value 6 Enabled if above MPC_LAND_ALT2 (except LANDING)
 *
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_INT32(VT_FWD_THRUST_EN, 0);

/**
 * Fixed-wing actuation thrust scale in hover
 *
 * Scale applied to the demanded pitch (below VT_PITCH_MIN) to get the fixed-wing forward actuation in hover mode.
 * Enabled via VT_FWD_THRUST_EN.
 *
 * @min 0.0
 * @max 5.0
 * @increment 0.01
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_FWD_THRUST_SC, 0.7f);

/**
 * Back transition MC motor ramp up time
 *
 * This sets the duration during which the MC motors ramp up to the commanded thrust during the back transition stage.
 *
 * @unit s
 * @min 0.0
 * @max 20.0
 * @increment 0.1
 * @decimal 1
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_B_TRANS_RAMP, 3.0f);

/**
 * Pusher throttle ramp up slew rate
 *
 * Defines the slew rate of the puller/pusher throttle during transitions.
 * Zero will deactivate the slew rate limiting and thus produce an instant throttle
 * rise to the transition throttle VT_F_TRANS_THR.
 *
 * @unit 1/s
 * @min 0
 * @increment 0.01
 * @decimal 2
 * @group VTOL Attitude Control
 */
PARAM_DEFINE_FLOAT(VT_PSHER_SLEW, 0.33f);
