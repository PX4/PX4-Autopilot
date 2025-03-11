/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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
 * @file pruepursuit_params.c
 *
 * Parameters defined by the pure pursuit lib.
 */

/**
 * Tuning parameter for the pure pursuit controller
 *
 * Lower value -> More aggressive controller (beware overshoot/oscillations)
 *
 * @min 0.1
 * @max 100
 * @increment 0.01
 * @decimal 2
 * @group Pure Pursuit
 */
PARAM_DEFINE_FLOAT(PP_LOOKAHD_GAIN, 1.0f);

/**
 * Minimum lookahead distance for the pure pursuit controller
 *
 * @min 0.1
 * @max 100
 * @unit m
 * @increment 0.01
 * @decimal 2
 * @group Pure Pursuit
 */
PARAM_DEFINE_FLOAT(PP_LOOKAHD_MIN, 1.0f);

/**
 * Maximum lookahead distance for the pure pursuit controller
 *
 * @min 0.1
 * @max 100
 * @unit m
 * @increment 0.01
 * @decimal 2
 * @group Pure Pursuit
 */
PARAM_DEFINE_FLOAT(PP_LOOKAHD_MAX, 10.0f);
