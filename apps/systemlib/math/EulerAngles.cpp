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
    Vector(3)
{
    phi() = 0.0f;
    theta() = 0.0f;
    psi() = 0.0f;
}

EulerAngles::EulerAngles(const Quaternion & q) :
    Vector(3)
{
    (*this) = EulerAngles(Dcm(q));
}

EulerAngles::EulerAngles(const Dcm & dcm) :
    Vector(3)
{
    theta() = asinf(-dcm(2,0)); 
    if (fabsf(theta() - M_PI_2_F) > 1.0e-3f)
    {
        phi() = 0.0f;
        psi() = atan2f(dcm(1,2) - dcm(0,1),
            dcm(0,2) + dcm(1,1)) + phi();
    }
    else if (fabsf(theta() + M_PI_2_F) > 1.0e-3f)
    {
        phi() = 0.0f;
        psi() = atan2f(dcm(1,2) - dcm(0,1),
            dcm(0,2) + dcm(1,1)) - phi();
    }
    else
    {
        phi() = atan2f(dcm(2,1),dcm(2,2));
        psi() = atan2f(dcm(1,0),dcm(0,0));
    }
}

} // namespace math
