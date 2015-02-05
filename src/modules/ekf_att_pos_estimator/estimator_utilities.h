/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file estimator_utilities.h
 *
 * Implementation of the attitude and position estimator.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Lorenz Meier <lm@inf.ethz.ch>
 */
#pragma once

#include <math.h>
#include <stdint.h>

#define GRAVITY_MSS 9.80665f
#define deg2rad 0.017453292f
#define rad2deg 57.295780f
#define pi 3.141592657f
#define earthRate 0.000072921f
#define earthRadius 6378145.0
#define earthRadiusInv  1.5678540e-7

class Vector3f
{
private:
public:
    float x;
    float y;
    float z;

    Vector3f(float a=0.0f, float b=0.0f, float c=0.0f) :
    x(a),
    y(b),
    z(c)
    {}

    float length(void) const;
    void zero(void);
};

class Mat3f
{
private:
public:
    Vector3f x;
    Vector3f y;
    Vector3f z;

    Mat3f();

    void identity();
    Mat3f transpose(void) const;
};

Vector3f operator*(float sclIn1, Vector3f vecIn1);
Vector3f operator+( Vector3f vecIn1, Vector3f vecIn2);
Vector3f operator-( Vector3f vecIn1, Vector3f vecIn2);
Vector3f operator*( Mat3f matIn, Vector3f vecIn);
Mat3f operator*( Mat3f matIn1, Mat3f matIn2);
Vector3f operator%( Vector3f vecIn1, Vector3f vecIn2);
Vector3f operator*(Vector3f vecIn1, float sclIn1);

void swap_var(float &d1, float &d2);

enum GPS_FIX {
    GPS_FIX_NOFIX = 0,
    GPS_FIX_2D = 2,
    GPS_FIX_3D = 3
};

struct ekf_status_report {
    bool error;
    bool velHealth;
    bool posHealth;
    bool hgtHealth;
    bool velTimeout;
    bool posTimeout;
    bool hgtTimeout;
    bool imuTimeout;
    bool onGround;
    bool staticMode;
    bool useCompass;
    bool useAirspeed;
    uint32_t velFailTime;
    uint32_t posFailTime;
    uint32_t hgtFailTime;
    float states[32];
    unsigned n_states;
    bool angNaN;
    bool summedDelVelNaN;
    bool KHNaN;
    bool KHPNaN;
    bool PNaN;
    bool covarianceNaN;
    bool kalmanGainsNaN;
    bool statesNaN;
    bool gyroOffsetsExcessive;
    bool covariancesExcessive;
    bool velOffsetExcessive;
};

void ekf_debug(const char *fmt, ...);
