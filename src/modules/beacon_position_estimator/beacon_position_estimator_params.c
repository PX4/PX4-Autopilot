/****************************************************************************
 *
 *   Copyright (c) 2014-2017 PX4 Development Team. All rights reserved.
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
 * @file beacon_position_estimator_params.c
 * Beacon estimator algorithm parameters.
 *
 * @author Nicolas de Palezieux <ndepal@gmail.com>
 */

/**
 * Acceleration uncertainty
 *
 * Variance of acceleration measurement used for beacon position prediction.
 * Higher values results in tighter following of the measurements and more lenient outlier rejection
 *
 * @unit (m/2^2)^2
 * @min 0.01
 * @decimal 2
 *
 * @group Beacon Estimator
 */
PARAM_DEFINE_FLOAT(BEST_ACC_UNC, 10f);

/**
 * Beacon measurement uncertainty
 *
 * Variance of the beacon measurement from the driver.
 * Higher values results in less agressive following of the measurement and a smoother output as well as fewer rejected measurements.
 *
 * @unit m^2
 * @decimal 4
 *
 * @group Beacon Estimator
 */
PARAM_DEFINE_FLOAT(BEST_MEAS_UNC, 0.01f);

/**
 * Initial beacon position uncertainty
 *
 * Initial variance of the relative beacon position in x and y direction
 *
 * @unit m^2
 * @min 0.001
 * @decimal 3
 *
 * @group Beacon Estimator
 */
PARAM_DEFINE_FLOAT(BEST_POS_UNC_IN, 0.1f);

/**
 * Initial beacon velocity uncertainty
 *
 * Initial variance of the relative beacon velocity in x and y direction
 *
 * @unit (m/s)^2
 * @min 0.001
 * @decimal 3
 *
 * @group Beacon Estimator
 */
PARAM_DEFINE_FLOAT(BEST_VEL_UNC_IN, 1.0f);
