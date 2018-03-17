/****************************************************************************
* Copyright (c) 2014, Paul Riseborough All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* Redistributions of source code must retain the above copyright notice, this
* list of conditions and the following disclaimer.
*
* Redistributions in binary form must reproduce the above copyright notice,
* this list of conditions and the following disclaimer in the documentation
* and/or other materials provided with the distribution.
*
* Neither the name of the {organization} nor the names of its contributors
* may be used to endorse or promote products derived from this software without
* specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
* ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
* LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
* INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
* CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
****************************************************************************/

/**
 * @file estimator_utilities.cpp
 *
 * Estimator support utilities.
 *
 * @author Paul Riseborough <p_riseborough@live.com.au>
 * @author Lorenz Meier <lorenz@px4.io>
 */

#include "estimator_utilities.h"

// Define EKF_DEBUG here to enable the debug print calls
// if the macro is not set, these will be completely
// optimized out by the compiler.
//#define EKF_DEBUG

#ifdef EKF_DEBUG
#include <stdio.h>
#include <stdarg.h>

static void
ekf_debug_print(const char *fmt, va_list args)
{
    fprintf(stderr, "%s: ", "[ekf]");
    vfprintf(stderr, fmt, args);

    fprintf(stderr, "\n");
}

void
ekf_debug(const char *fmt, ...)
{
    va_list args;

    va_start(args, fmt);
    ekf_debug_print(fmt, args);
}

#else

void ekf_debug(const char *fmt, ...) { while(0){} }
#endif

/* we don't want to pull in the standard lib just to swap two floats */
void swap_var(float &d1, float &d2);

float Vector3f::length() const
{
    return sqrtf(x*x + y*y + z*z);
}

void Vector3f::zero()
{
    x = 0.0f;
    y = 0.0f;
    z = 0.0f;
}

Mat3f::Mat3f() :
    x{1.0f, 0.0f, 0.0f},
    y{0.0f, 1.0f, 0.0f},
    z{0.0f, 0.0f, 1.0f}
{
}

void Mat3f::identity() {
    x.x = 1.0f;
    x.y = 0.0f;
    x.z = 0.0f;

    y.x = 0.0f;
    y.y = 1.0f;
    y.z = 0.0f;

    z.x = 0.0f;
    z.y = 0.0f;
    z.z = 1.0f;
}

Mat3f Mat3f::transpose() const
{
    Mat3f ret = *this;
    swap_var(ret.x.y, ret.y.x);
    swap_var(ret.x.z, ret.z.x);
    swap_var(ret.y.z, ret.z.y);
    return ret;
}

void calcvelNED(float (&velNEDr)[3], float gpsCourse, float gpsGndSpd, float gpsVelD)
{
    velNEDr[0] = gpsGndSpd*cosf(gpsCourse);
    velNEDr[1] = gpsGndSpd*sinf(gpsCourse);
    velNEDr[2] = gpsVelD;
}

void calcposNED(float (&posNEDr)[3], double lat, double lon, float hgt, double latReference, double lonReference, float hgtReference)
{
    posNEDr[0] = earthRadius * (lat - latReference);
    posNEDr[1] = earthRadius * cos(latReference) * (lon - lonReference);
    posNEDr[2] = -(hgt - hgtReference);
}

void calcLLH(float posNEDi[3], double &lat, double &lon, float &hgt, double latRef, double lonRef, float hgtRef)
{
    lat = latRef + (double)posNEDi[0] * earthRadiusInv;
    lon = lonRef + (double)posNEDi[1] * earthRadiusInv / cos(latRef);
    hgt = hgtRef - posNEDi[2];
}

// overload + operator to provide a vector addition
Vector3f operator+(const Vector3f &vecIn1, const Vector3f &vecIn2)
{
    Vector3f vecOut;
    vecOut.x = vecIn1.x + vecIn2.x;
    vecOut.y = vecIn1.y + vecIn2.y;
    vecOut.z = vecIn1.z + vecIn2.z;
    return vecOut;
}

// overload - operator to provide a vector subtraction
Vector3f operator-(const Vector3f &vecIn1, const Vector3f &vecIn2)
{
    Vector3f vecOut;
    vecOut.x = vecIn1.x - vecIn2.x;
    vecOut.y = vecIn1.y - vecIn2.y;
    vecOut.z = vecIn1.z - vecIn2.z;
    return vecOut;
}

// overload * operator to provide a matrix vector product
Vector3f operator*(const Mat3f &matIn, const Vector3f &vecIn)
{
    Vector3f vecOut;
    vecOut.x = matIn.x.x*vecIn.x + matIn.x.y*vecIn.y + matIn.x.z*vecIn.z;
    vecOut.y = matIn.y.x*vecIn.x + matIn.y.y*vecIn.y + matIn.y.z*vecIn.z;
    vecOut.z = matIn.z.x*vecIn.x + matIn.z.y*vecIn.y + matIn.z.z*vecIn.z;
    return vecOut;
}

// overload * operator to provide a matrix product
Mat3f operator*(const Mat3f &matIn1, const Mat3f &matIn2)
{
    Mat3f matOut;
    matOut.x.x = matIn1.x.x*matIn2.x.x + matIn1.x.y*matIn2.y.x + matIn1.x.z*matIn2.z.x;
    matOut.x.y = matIn1.x.x*matIn2.x.y + matIn1.x.y*matIn2.y.y + matIn1.x.z*matIn2.z.y;
    matOut.x.z = matIn1.x.x*matIn2.x.z + matIn1.x.y*matIn2.y.z + matIn1.x.z*matIn2.z.z;

    matOut.y.x = matIn1.y.x*matIn2.x.x + matIn1.y.y*matIn2.y.x + matIn1.y.z*matIn2.z.x;
    matOut.y.y = matIn1.y.x*matIn2.x.y + matIn1.y.y*matIn2.y.y + matIn1.y.z*matIn2.z.y;
    matOut.y.z = matIn1.y.x*matIn2.x.z + matIn1.y.y*matIn2.y.z + matIn1.y.z*matIn2.z.z;

    matOut.z.x = matIn1.z.x*matIn2.x.x + matIn1.z.y*matIn2.y.x + matIn1.z.z*matIn2.z.x;
    matOut.z.y = matIn1.z.x*matIn2.x.y + matIn1.z.y*matIn2.y.y + matIn1.z.z*matIn2.z.y;
    matOut.z.z = matIn1.z.x*matIn2.x.z + matIn1.z.y*matIn2.y.z + matIn1.z.z*matIn2.z.z;

    return matOut;
}

// overload % operator to provide a vector cross product
Vector3f operator%(const Vector3f &vecIn1, const Vector3f &vecIn2)
{
    Vector3f vecOut;
    vecOut.x = vecIn1.y*vecIn2.z - vecIn1.z*vecIn2.y;
    vecOut.y = vecIn1.z*vecIn2.x - vecIn1.x*vecIn2.z;
    vecOut.z = vecIn1.x*vecIn2.y - vecIn1.y*vecIn2.x;
    return vecOut;
}

// overload * operator to provide a vector scaler product
Vector3f operator*(const Vector3f &vecIn1, const float sclIn1)
{
    Vector3f vecOut;
    vecOut.x = vecIn1.x * sclIn1;
    vecOut.y = vecIn1.y * sclIn1;
    vecOut.z = vecIn1.z * sclIn1;
    return vecOut;
}

// overload * operator to provide a vector scaler product
Vector3f operator*(float sclIn1, const Vector3f &vecIn1)
{
    Vector3f vecOut;
    vecOut.x = vecIn1.x * sclIn1;
    vecOut.y = vecIn1.y * sclIn1;
    vecOut.z = vecIn1.z * sclIn1;
    return vecOut;
}

// overload / operator to provide a vector scalar division
Vector3f operator/(const Vector3f &vec, const float scalar)
{
    Vector3f vecOut;
    vecOut.x = vec.x / scalar;
    vecOut.y = vec.y / scalar;
    vecOut.z = vec.z / scalar;
    return vecOut;
}

void swap_var(float &d1, float &d2)
{
    float tmp = d1;
    d1 = d2;
    d2 = tmp;
}
