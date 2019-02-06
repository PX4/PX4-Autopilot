/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
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
 * @file sih_params.c
 * Parameters for quadcopter simulator in hardware.
 *
 * @author Romain Chiappinelli <romain.chiap@gmail.com>
 * January 2019
 */

// ================== Vehicle physical properties ======================== 

// vehicle mass [kg]
PARAM_DEFINE_FLOAT(SIH_MASS, 1.0f);

// vehicle Inertia matrix [kg*m^2]
//     [Ixx, Ixy, Ixz]
// I = [Ixy, Iyy, Iyz]
//     [Ixz, Iyz, Izz]
PARAM_DEFINE_FLOAT(SIH_IXX, 0.025f);
PARAM_DEFINE_FLOAT(SIH_IYY, 0.025f);
PARAM_DEFINE_FLOAT(SIH_IZZ, 0.030f);
PARAM_DEFINE_FLOAT(SIH_IXY, 0.0f); 
PARAM_DEFINE_FLOAT(SIH_IXZ, 0.0f);
PARAM_DEFINE_FLOAT(SIH_IYZ, 0.0f);

// maximum thrust of one propeller [N]
PARAM_DEFINE_FLOAT(SIH_T_MAX, 5.0f);

// maximum torque of one propeller [Nm]
PARAM_DEFINE_FLOAT(SIH_Q_MAX, 0.1f);

// arm length generating the rolling moment [m] 
// (i.e. distance from the left motors to the CM)
PARAM_DEFINE_FLOAT(SIH_L_ROLL, 0.2f);

// arm length generating the pitching moment [m] 
// (i.e. the distance from the front motors to the CM)
PARAM_DEFINE_FLOAT(SIH_L_PITCH, 0.2f);

// First order drag coefficient [N/(m/s)]
// Drag force function of velocity: D=-KDV*V 
PARAM_DEFINE_FLOAT(SIH_KDV, 1.0f);

// First order angular damper coefficient [Nm/(rad/s)]
// Aerodynamic moment function of body rate: Ma=-KDW*W_B 
PARAM_DEFINE_FLOAT(SIH_KDW, 0.025f);

// ================== Vehicle initial location ========================

// initial geodetic latitude [1e-7 deg]
PARAM_DEFINE_INT32(SIH_LAT0, 454671160);
// initial geodetic longitude [1e-7 deg]
PARAM_DEFINE_INT32(SIH_LON0, -737578370);
// ground altitude at this location [m]
PARAM_DEFINE_FLOAT(SIH_H0, 32.34f);

// NED Magnetic field at this location [G]
PARAM_DEFINE_FLOAT(SIH_MU_X,  0.2903f);
PARAM_DEFINE_FLOAT(SIH_MU_Y, -0.0832f);
PARAM_DEFINE_FLOAT(SIH_MU_Z,  0.950f);
