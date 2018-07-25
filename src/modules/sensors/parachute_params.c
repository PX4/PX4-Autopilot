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
 * @file parachute_params.c
 *
 * Parameters for parachute triggering.
 *
 * @author Mathieu Bresciani <brescianimathieu@gmail.com>
 *
 */


/**
 * Parachute output channel.
 *
 * Output where the signal to the parachute trigger is connected
 *
 * Zero means that parachute triggering is disabled.
 *
 * @value 0 Disabled
 * @value 1 IO1
 * @value 2 IO2
 * @value 3 IO3
 * @value 4 IO4
 * @value 5 IO5
 * @value 6 IO6
 * @value 7 IO7
 * @value 8 IO8
 * @value 9 FMU1
 * @value 10 FMU2
 * @value 11 FMU3
 * @value 12 FMU4
 * @value 13 FMU5
 * @value 14 FMU6
 * @value 15 FMU7
 * @value 16 FMU8
 * @min 0
 * @max 16
 * @decimal 0
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_CHUTE_OUT, 0);

/**
 * Parachute OFF value.
 *
 * PWM value that has to be sent to the parachute during normal operation (no deployment).
 *
 * @min 800
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_CHUTE_OFF, 1000);

/**
 * Parachute ON value.
 *
 * PWM value that has to be sent to the parachute during failsafe operation (deployment).
 *
 * @min 800
 * @max 2200
 * @unit us
 * @group PWM Outputs
 */
PARAM_DEFINE_INT32(PWM_CHUTE_ON, 2000);
