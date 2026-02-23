/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
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
 * Enable Actuator Failure check
 *
 * If enabled, the HealthAndArmingChecks will verify that for motors, a minimum amount of ESC current per throttle
 * level is being consumed.
 * Otherwise this indicates an motor failure.
 * This check only works for ESCs that report current consumption.
 *
 * @boolean
 *
 * @group Motor Failure
 */
PARAM_DEFINE_INT32(FD_ACT_EN, 0);

/**
 * Motor Failure Current/Throttle Scale
 *
 * Determines the slope between expected steady state current and linearized, normalized thrust command.
 * E.g. FD_ACT_MOT_C2T A represents the expected steady state current at 100%.
 * FD_ACT_LOW_OFF and FD_ACT_HIGH_OFF offset the threshold from that slope.
 *
 * @group Motor Failure
 * @min 0.0
 * @max 50.0
 * @unit A/%
 * @decimal 2
 * @increment 1
 */
PARAM_DEFINE_FLOAT(MOTFAIL_C2T, 35.f);

/**
 * Undercurrent motor failure limit offset
 *
 * threshold = FD_ACT_MOT_C2T * thrust - FD_ACT_LOW_OFF
 *
 * @group Motor Failure
 * @min 0
 * @max 30
 * @unit A
 * @decimal 2
 * @increment 1
 */
PARAM_DEFINE_FLOAT(MOTFAIL_LOW_OFF, 10.f);

/**
 * Overcurrent motor failure limit offset
 *
 * threshold = FD_ACT_MOT_C2T * thrust + FD_ACT_HIGH_OFF
 *
 * @group Motor Failure
 * @min 0
 * @max 30
 * @unit A
 * @decimal 2
 * @increment 1
 */
PARAM_DEFINE_FLOAT(MOTFAIL_HIGH_OFF, 10.f);

/**
 * Motor Failure Hysteresis Time
 *
 * Motor failure only triggers after current thresholds are exceeded for this time.
 *
 * @group Motor Failure
 * @unit ms
 * @min 10
 * @max 10000
 * @increment 100
 */
PARAM_DEFINE_INT32(MOTFAIL_TIME, 1000);
