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
#include <drivers/drv_hrt.h>

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

    VectorType gyroB(3);
    VectorType accelB(3);
    VectorType zGps(6);
    VectorType zMag(3);

    uint64_t old_timestamp = hrt_absolute_time();
    uint16_t nav_frames = 0;
    float dt = 0.01;

    printf("\n");

    for (int i=0;i<1000;i++)
    {
        gyroB(0)  = 0.1f*dt;
        gyroB(1)  = 0.1f*dt;
        gyroB(2)  = 0.1f*dt;
        accelB(0) = 0.1f*dt;
        accelB(1) = 0.1f*dt;
        accelB(2) = 0.1f*dt;

        if (i%10==0)
        {
            zGps(0) = 0.1f; // vn
            zGps(1) = 0.1f; // ve
            zGps(2) = 0.1f; // vd
            zGps(3) = 0.1f; // L
            zGps(4) = 0.1f; // l
            zGps(5) = 0.1f; // h
            nav.correctGps(zGps);
        }
        if (i%5==0)
        {
            zMag(0) = 0.1f;
            zMag(1) = 0.1f;
            zMag(2) = 0.1f;
            nav.correctMag(zMag);
        }
        nav.predict(0.01,accelB,gyroB);

        // output
        uint64_t timestamp = hrt_absolute_time();
        nav_frames += 1;
        if ((timestamp - old_timestamp) > 1000000)
        {
            printf("nav running at %d hz\n", nav_frames);
            nav.getX().print();
            old_timestamp = timestamp;
            nav_frames = 0;
        }
    }
    printf("PASS\n");
    return 0;
}

} // namespace math
