/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * Enable nudging based on user input during autonomous land routine
 *
 * Using stick input the vehicle can be moved horizontally and yawed.
 * The descend speed is amended:
 * stick full up - 0
 * stick centered - MPC_LAND_SPEED
 * stick full down - 2 * MPC_LAND_SPEED
 *
 * Manual override during auto modes has to be disabled to use this feature (see COM_RC_OVERRIDE).
 *
 * @min 0
 * @max 1
 * @value 0 Nudging disabled
 * @value 1 Nudging enabled
 * @group Multicopter Position Control
 */
PARAM_DEFINE_INT32(MPC_LAND_RC_HELP, 0);

/**
 * User assisted landing radius
 *
 * When nudging is enabled (see MPC_LAND_RC_HELP), this defines the maximum
 * allowed horizontal displacement from the original landing point. Nudging in
 * FlightTaskAuto is then restricted as follows:
 *  - If the vehicle is inside of the allowed radius, only allow nudging inputs that do not move the vehicle outside of it.
 *  - If the vehicle is outside of the allowed radius, only allow nudging inputs that move the vehicle back towards it.
 *
 * Setting MPC_LAND_RADIUS to -1 will disable this feature.
 *
 * @unit m
 * @min -1
 * @decimal 0
 * @increment 1
 * @group Multicopter Position Control
 */
PARAM_DEFINE_FLOAT(MPC_LAND_RADIUS, -1.0f);
