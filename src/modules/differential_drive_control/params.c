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
 * Wheel Base
 *
 * Distance from the center of the right wheel to the center of the left wheel
 *
 * @unit m
 * @min 0.001
 * @max 100
 * @increment 0.001
 * @decimal 3
 * @group Rover Differential Drive
 */
PARAM_DEFINE_FLOAT(RDD_WHEEL_BASE, 0.5f);

/**
 * Wheel radius
 *
 * Size of the wheel, half the diameter of the wheel
 *
 * @unit m
 * @min 0.001
 * @max 100
 * @increment 0.001
 * @decimal 3
 * @group Rover Differential Drive
 */
PARAM_DEFINE_FLOAT(RDD_WHEEL_RADIUS, 0.1f);

/**
 * Max Speed
 *
 * @unit m/s
 * @min 0.0
 * @max 100
 * @increment 0.01
 * @decimal 2
 * @group Rover Differential Drive
 */
PARAM_DEFINE_FLOAT(RDD_MAX_SPEED, 0.5f);

/**
 * Max Angular Velocity
 *
 * @unit rad/s
 * @min 0.0
 * @max 100
 * @increment 0.01
 * @decimal 2
 * @group Rover Differential Drive
 */
PARAM_DEFINE_FLOAT(RDD_MAX_ANG_VEL, 0.3f);
