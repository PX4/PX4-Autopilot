/****************************************************************************
 *
 *   Copyright (C) 2012 PX4 Development Team. All rights reserved.
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
 * @file Quaternion.cpp
 *
 * math vector
 */

#include <systemlib/test/test.hpp>


#include "Quaternion.hpp"
#include "Dcm.hpp"
#include "EulerAngles.hpp"

namespace math
{

Quaternion::Quaternion() :
    Vector(4),
    a((*this)(0)),
    b((*this)(1)),
    c((*this)(2)),
    d((*this)(3))
{
    a = 1.0f;
    b = 0.0f;
    c = 0.0f;
    d = 0.0f;
}

Quaternion::Quaternion(const float * data) :
    Vector(4,data),
    a((*this)(0)),
    b((*this)(1)),
    c((*this)(2)),
    d((*this)(3))
{
}

Quaternion::Quaternion(const Dcm & dcm) :
    Vector(4),
    a((*this)(0)),
    b((*this)(1)),
    c((*this)(2)),
    d((*this)(3))
{
    // TODO
    a = 1.0f;
    b = 0.0f;
    c = 0.0f;
    d = 0.0f;
}

Quaternion::Quaternion(const EulerAngles & euler) :
    Vector(4),
    a((*this)(0)),
    b((*this)(1)),
    c((*this)(2)),
    d((*this)(3))
{
    // initialize quaternions
    float cosPhi_2 = cosf(euler.phi/2.0f);
    float cosTheta_2 = cosf(euler.theta/2.0f);
    float cosPsi_2 = cosf(euler.psi/2.0f);
    float sinPhi_2 = sinf(euler.phi/2.0f);
    float sinTheta_2 = sinf(euler.theta/2.0f);
    float sinPsi_2 = sinf(euler.psi/2.0f);
    a = cosPhi_2*cosTheta_2*cosPsi_2 + 
        sinPhi_2*sinTheta_2*sinPsi_2;
    b = sinPhi_2*cosTheta_2*cosPsi_2 -
        cosPhi_2*sinTheta_2*sinPsi_2;
    c = cosPhi_2*sinTheta_2*cosPsi_2 +
        sinPhi_2*cosTheta_2*sinPsi_2;
    d = cosPhi_2*cosTheta_2*sinPsi_2 +
        sinPhi_2*sinTheta_2*cosPsi_2;
}

Quaternion::Quaternion(const Quaternion & right) :
    Vector(right),
    a((*this)(0)),
    b((*this)(1)),
    c((*this)(2)),
    d((*this)(3))
{
}

Quaternion::~Quaternion()
{
}

Vector Quaternion::derivative(const Vector & w)
{
#ifdef QUATERNION_ASSERT
    ASSERT(w.getRows()==3);
#endif
    float dataQ[] = 
    {a, -b, -c, -d,
     b,  a, -d,  c,
     c,  d,  a, -b,
     d, -c,  b,  a};
    Vector v(4);
    v(0) = 0.0f;
    v(1) = w(0);
    v(2) = w(1);
    v(3) = w(2);
    Matrix Q(4,4,dataQ);
    return Q*v*0.5f; 
}

int __EXPORT quaternionTest()
{
    return 0;
}

} // namespace math
