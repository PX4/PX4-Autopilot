/****************************************************************************
 *
 *   Copyright (c) 2013-2019 PX4 Development Team. All rights reserved.
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
 * @file vehicle_model_estimator_params.c
 * Parameters for the estimation of the vehicle model.
 *
 * @author Julien Lecoeur <julien.lecoeur@gmail.com>
 */


/**
 * Enables the vehicle model estimation
 *
 * @boolean
 * @group Vehicle Model
 */
PARAM_DEFINE_INT32(VM_EST_EN, 1);

/**
 * Estimated vehicle model publishing rate
 *
 * The estimator will always run at maximum rate, updating its
 * estimate at full rate.
 * This parameter may be used to limit the rate at which the
 * resulting estimation is published.
 *
 * @unit Hz
 * @min 0
 * @max 1000
 * @decimal 0
 * @increment 1
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_EST_PUBRATE, 10.0f);

/**
 * Estimation gain
 *
 * A larger value will lead to faster update of the estimated model
 * but an excessively large value may cause faulty estimation.
 *
 * @decimal 5
 * @increment 0.00001
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_EST_GAIN, 0.1f);

/**
 * Vehicle model estimator cutoff frequency
 *
 * Determines the cutoff frequency of the 2nd order low pass filter
 * applied on control setpoints and sensor data before estimation.
 *
 * @min 0
 * @max 30
 * @decimal 2
 * @increment 1
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_EST_CUTOFF, 5.0f);

/**
 * Vehicle model estimator actuator delay
 *
 * Determines the delay applied to the torque and thrust setpoints
 * in order to take into account the time required for the
 * setpoints to take effect.
 * This delay may vary depending on
 * - whether the setpoints go through an IO board
 * - the latency of the protocol used to communicate with actuators
 * - the intrinsic latency of actuators
 *
 * @unit s
 * @min 0
 * @max 0.5
 * @decimal 4
 * @increment 1
 * @group Vehicle Model
 */
PARAM_DEFINE_FLOAT(VM_EST_DELAY, 0.080f);
