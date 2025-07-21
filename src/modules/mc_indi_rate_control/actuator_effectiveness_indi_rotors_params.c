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
 * @file actuator_effectiveness_indi_rotors_params.c
 *
 * Parameters for INDI actuator effectiveness rotors
 */

/**
 * INDI Adaptive Constant - Roll Axis
 *
 * Adaptive constant for the roll axis in the INDI effectiveness matrix adaptation.
 * Larger values result in faster adaptation but can cause instability if too high.
 * Based on the paper "Adaptive Incremental Nonlinear Dynamic Inversion for Attitude Control of Micro Air Vehicles" by Smeur et al.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @unit 1
 */
PARAM_DEFINE_FLOAT(INDI_ADAPT_ROLL, 1.5f);

/**
 * INDI Adaptive Constant - Pitch Axis
 *
 * Adaptive constant for the pitch axis in the INDI effectiveness matrix adaptation.
 * Larger values result in faster adaptation but can cause instability if too high.
 * Based on the paper "Adaptive Incremental Nonlinear Dynamic Inversion for Attitude Control of Micro Air Vehicles" by Smeur et al.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @unit 1
 */
PARAM_DEFINE_FLOAT(INDI_ADAPT_PITCH, 1.5f);

/**
 * INDI Adaptive Constant - Yaw Axis
 *
 * Adaptive constant for the yaw axis in the INDI effectiveness matrix adaptation.
 * Larger values result in faster adaptation but can cause instability if too high.
 * Based on the paper "Adaptive Incremental Nonlinear Dynamic Inversion for Attitude Control of Micro Air Vehicles" by Smeur et al.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @unit 1
 */
PARAM_DEFINE_FLOAT(INDI_ADAPT_YAW, 1.5f);
