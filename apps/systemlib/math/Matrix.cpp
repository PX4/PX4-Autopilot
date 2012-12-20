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
 * @file Matrix.cpp
 *
 * matrix code
 */

#include <systemlib/test/test.hpp>
#include <math.h>

#include "Matrix.hpp"

namespace math
{

static const float data_a[] = 
    {1,2,3,
     4,5,6};
static const float data_b[] = 
    {0,1,3,
     7,-1,2};
static const float data_c[] = 
    {0,1,
     2,1,
     3,2};
static const float data_d[] = 
    {0,1,2,
     2,1,4,
     5,2,0};
static const float data_e[] = 
    {1,-1,2,
     0,2,3,
     2,-1,1};
static const float data_f[] = 
    {3.777e006f, 2.915e007f, 0.000e000f,
     2.938e007f, 2.267e008f, 0.000e000f,
     0.000e000f, 0.000e000f, 6.033e008f};

static MatrixFloat a(2,3,data_a);
static MatrixFloat b(2,3,data_b);
static MatrixFloat c(3,2,data_c);
static MatrixFloat d(3,3,data_d);
static MatrixFloat e(3,3,data_e);
static MatrixFloat f(3,3,data_f);

int __EXPORT matrixTest()
{
    matrixAddTest();
    matrixSubTest();
    matrixMultTest();
    matrixInvTest();
    matrixDivTest();
    return 0;
}

int matrixAddTest()
{
    printf("Test Matrix Add\t\t: ");
    MatrixFloat r = a + b;
    ASSERT(equal(r(0,0),1.0f))
    ASSERT(equal(r(0,1),3.0f))
    ASSERT(equal(r(0,2),6.0f))
    ASSERT(equal(r(1,0),11.0f))
    ASSERT(equal(r(1,1),4.0f))
    ASSERT(equal(r(1,2),8.0f))
    printf("PASS\n");
    return 0;
}

int matrixSubTest()
{
    printf("Test Matrix Sub\t\t: ");
    MatrixFloat r = a - b;
    ASSERT(equal(r(0,0),1.0f))
    ASSERT(equal(r(0,1),1.0f))
    ASSERT(equal(r(0,2),0.0f))
    ASSERT(equal(r(1,0),-3.0f))
    ASSERT(equal(r(1,1),6.0f))
    ASSERT(equal(r(1,2),4.0f))
    printf("PASS\n");
    return 0;
}

int matrixMultTest()
{
    printf("Test Matrix Mult\t: ");
    MatrixFloat r = c * b;
    ASSERT(equal(r(0,0),7.0f))
    ASSERT(equal(r(0,1),-1.0f))
    ASSERT(equal(r(0,2),2.0f))
    ASSERT(equal(r(1,0),7.0f))
    ASSERT(equal(r(1,1),1.0f))
    ASSERT(equal(r(1,2),8.0f))
    ASSERT(equal(r(2,0),14.0f))
    ASSERT(equal(r(2,1),1.0f))
    ASSERT(equal(r(2,2),13.0f))
    printf("PASS\n");
    return 0;
}

int matrixInvTest()
{
    printf("Test Matrix Inv\t\t: ");
    MatrixFloat r = f.inverse();
    ASSERT(equal(r(0,0),-0.0012518f,1e-6f))
    ASSERT(equal(r(0,1),0.0001610f,1e-6f))
    ASSERT(equal(r(0,2),0.0000000f,1e-6f))
    ASSERT(equal(r(1,0),0.0001622f,1e-6f))
    ASSERT(equal(r(1,1),-0.0000209f,1e-6f))
    ASSERT(equal(r(1,2),0.0000000f,1e-6f))
    ASSERT(equal(r(2,0),0.0000000f,1e-6f))
    ASSERT(equal(r(2,1),0.0000000f,1e-6f))
    ASSERT(equal(r(2,2),1.6580e-9f,1e-6f))
    printf("PASS\n");
    return 0;
}

int matrixDivTest()
{
    printf("Test Matrix Div\t\t: ");
    MatrixFloat r = d / e;
    ASSERT(equal(r(0,0),0.2222222f,1e-6f))
    ASSERT(equal(r(0,1),0.5555556f,1e-6f))
    ASSERT(equal(r(0,2),-0.1111111f,1e-6f))
    ASSERT(equal(r(1,0),0.0f,1e-6f))
    ASSERT(equal(r(1,1),1.0f,1e-6f))
    ASSERT(equal(r(1,2),1.0f,1e-6f))
    ASSERT(equal(r(2,0),-4.1111111f,1e-6f))
    ASSERT(equal(r(2,1),1.2222222f,1e-6f))
    ASSERT(equal(r(2,2),4.5555556f,1e-6f))
    printf("PASS\n");
    return 0;
}

} // namespace math
