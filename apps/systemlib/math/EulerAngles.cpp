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
 * @file Vector.cpp
 *
 * math vector
 */

#include <systemlib/test/test.hpp>

#include "EulerAngles.hpp"
#include "Quaternion.hpp"
#include "Dcm.hpp"

namespace math
{

EulerAngles::EulerAngles() :
    Vector(3),
    phi((*this)(0)),
    theta((*this)(1)),
    psi((*this)(2))
{
    phi = 0.0f;
    theta = 0.0f;
    psi = 0.0f;
}

EulerAngles::EulerAngles(const Quaternion & q) :
    Vector(3),
    phi((*this)(0)),
    theta((*this)(1)),
    psi((*this)(2))
{
    float a = q.a;
    float b = q.b;
    float c = q.c;
    float d = q.d;
    float aSq = a*a;
    float bSq = b*b;
    float cSq = c*c;
    float dSq = d*d;
    theta = asinf(2*(a*c - b*d));
    phi  = atan2f(2*(a*b + c*d),
            aSq - bSq - cSq + dSq);
    psi = atan2f(2*(a*d + b*c),
            aSq + bSq - cSq - dSq);
}

EulerAngles::EulerAngles(const Dcm & dcm) :
    Vector(3),
    phi((*this)(0)),
    theta((*this)(1)),
    psi((*this)(2))
{
    // TODO
}

} // namespace math
