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
 * @file Kalman.cpp
 *
 * kalman filter code
 */

#include "Kalman.hpp"

namespace math
{

int __EXPORT kalmanTest()
{
    kalmanNavTest();
    return 0;
}

int __EXPORT kalmanNavTest()
{
    printf("Test Kalman Nav\t\t: ");
    typedef Matrix<float> MatrixType;
    typedef Vector<float> VectorType;
    KalmanNav<float> nav;
    VectorType accelB(3);
    accelB.setAll(1.0f);
    VectorType gyroB(3);
    gyroB.setAll(1.0f);
    for (int i=0;i<100;i++)
    {
        if (i%10==0)
        {
            Vector<float> zGps(6);
            zGps.setAll(1);
            nav.correctGps(zGps);
        }
        if (i%5==0)
        {
            Vector<float> zMag(3);
            zMag.setAll(1);
            nav.correctMag(zMag);
        }
        nav.predict(0.01,accelB,gyroB);
        nav.getX().print();
    }
    printf("PASS\n");
    return 0;
}

} // namespace math
