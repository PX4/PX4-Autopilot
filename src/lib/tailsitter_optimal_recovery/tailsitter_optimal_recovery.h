/****************************************************************************
 *
 *	Copyright (c) 2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in
 *	the documentation and/or other materials provided with the
 *	distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *	used to endorse or promote products derived from this software
 *	without specific prior written permission.
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
 * @file tailsitter_optimal_recovery.h
 *
 * Tailsitter optimal rate controller (underactuated pitch axis)

 * This function can be used to compute desired body rates of a tailsitter
 * with an underactuated pitch axis. Tailsitters which produce a pitching moment
 * from airflow over control surfaces mostly suffer from an underactuated pitch axis.
 * This functions captures the solution of an optimal control problem which minimises
 * the vehicle's tilt error and which penalises the desired rates of the underactuated
 * pitch axis.
 * Publication:
 * Robin Ritz and Raffaello D'Andrea. A Global Strategy for Tailsitter Hover Control.
 *
 * @author Roman Bapst <bapstroman@gmail.com>
 */

#include <lib/mathlib/mathlib.h>

// optimal coefficients
static const float pwx_kx1 = -1.676f;
static const float pwx_ky1 = 1.38f;
static const float pwx_ky2 = 0.8725f;
static const float pwx_sx0 = 0.3586f;
static const float pwx_sx1 = 2.642f;

static const float pwy_kx1 = -3.997f;
static const float pwy_sx0 = 2.133f;
static const float pwy_sx1 = 4.013f;

static const float pwz_kx1 = 2.726f;
static const float pwz_ky1 = 3.168f;
static const float pwz_ky2 = -0.3913f;
static const float pwz_sx0 = 1.75f;
static const float pwz_sx1 = 2.298f;


__EXPORT void computeOptimalRates(const math::Quaternion &q, const math::Quaternion &q_sp, float *_rates_sp);
