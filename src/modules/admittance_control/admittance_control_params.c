/****************************************************************************
 *
 *   Copyright (c) 2020 PX4 Development Team. All rights reserved.
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
 * @file admittance_control_params.c
 *
 * Parameters used by the Admittance Controller
 *
 * @author Pedro Mendes <pmen817@aucklanduni.ac.nz>
 */

/**
 * Ax Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_AX, 2.5f);

/**
 * Ay Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_AY, 2.5f);

/**
 * Az Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_AZ, 2.5f);


/**
 * Ayaw Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_AW, 2.5f);

/**
 * B1x Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_B1X, 5.f);

/**
 * B1y Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_B1Y, 5.f);

/**
 * B1z Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_B1Z, 5.f);


/**
 * B1yaw Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_B1W, 5.f);


/**
 * B2x Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_B2X, 3.5f);

/**
 * B2y Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_B2Y, 3.5f);

/**
 * B2z Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_B2Z, 3.5f);


/**
 * B2yaw Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_B2W, 3.5f);


/**
 * B3x Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_B3X, 2.f);

/**
 * B3y Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_B3Y, 2.f);

/**
 * B3z Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_B3Z, 2.f);


/**
 * B3yaw Bell Curve
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_B3W, 2.f);


/**
 * Min. mass x-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_MIX, 0.5f);

/**
 * Min. mass y-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_MIY, 1.f);

/**
 * Min. mass z-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_MIZ, 1.f);


/**
 * Min. mass yaw-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_MIW, 10.f);

/**
 * Min. stiffness x-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_KIX, 1.f);

/**
 * Min.stiffness y-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_KIY, 10.f);

/**
 * Min. stiffness z-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_KIZ, 10.f);


/**
 * Min. stiffness yaw-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_KIW, 0.1f);

/**
 * Max. mass x-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_MAX, 1.f);

/**
 * Max. mass y-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_MAY, 10.f);

/**
 * Max. mass z-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_MAZ, 10.f);


/**
 * Max. mass yaw-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 150.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_MAW, 100.f);

/**
 * Max. stiffness x-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_KAX, 2.f);

/**
 * Max.stiffness y-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_KAY, 25.f);

/**
 * Max. stiffness z-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_KAZ, 25.f);


/**
 * Max. stiffness yaw-axis
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_KAW, 1.f);

/**
 * Saturation Factor Time Constant
 *
 * @decimal 5
 * @min 0.01
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_BEL_LPF, 5.f);

/**
 * External Force Deadzone x-axis
 *
 * @decimal 5
 * @min 0.001
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_WRE_DZX, 0.125f);

/**
 * External Force Deadzone y-axis
 *
 * @decimal 5
 * @min 0.001
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_WRE_DZY, 0.125f);

/**
 * External Force Deadzone z-axis
 *
 * @decimal 5
 * @min 0.001
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_WRE_DZZ, 0.175f);

/**
 * External Moment Deadzone yaw-axis
 *
 * @decimal 6
 * @min 0.0001
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_WRE_DZW, 0.0350f);

/**
 * External Force Saturation x-axis
 *
 * @decimal 5
 * @min 0.001
 * @max 100.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_WRE_SAX, 2.5f);

/**
 * External Force Saturation y-axis
 *
 * @decimal 5
 * @min 0.001
 * @max 100.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_WRE_SAY, 2.5f);

/**
 * External Force Saturation z-axis
 *
 * @decimal 5
 * @min 0.001
 * @max 100.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_WRE_SAZ, 15.f);

/**
 * External Moment Saturation yaw-axis
 *
 * @decimal 6
 * @min 0.0001
 * @max 50.0
 * @group Admittance Control
 */
PARAM_DEFINE_FLOAT(ADM_CTR_WRE_SAW, 0.5f);
