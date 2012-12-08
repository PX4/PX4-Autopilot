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
 * @file blocks.cpp
 *
 * Controller library code
 */

#include <math.h>
#include <stdio.h>

#include "blocks.h"

namespace control
{

int basicBlocksTest()
{
    blockLimitTest();
    blockLimitSymTest();
    blockLowPassTest();
    blockHighPassTest();
    return 0;
}

bool equal(float a, float b, float epsilon = 1e-7)
{
    float diff = fabs(a-b);
    if (diff>epsilon)
    {
        printf("not equal ->\n\ta: %12.8f\n\tb: %12.8f\n", double(a), double(b)); 
        return false;
    }
    else return true;
}

float BlockLimit::update(float input)
{
    if (input > getMax())
    {
        input = _max.get();
    }
    else if (input < getMin())
    {
        input = getMin();
    }
    return input;
}

int blockLimitTest()
{
    BlockLimit limit(NULL,"TEST");
    // initial state
    ASSERT(equal(1.0f,limit.getMax()));
    ASSERT(equal(-1.0f,limit.getMin()));
    ASSERT(equal(0.0f,limit.getDt()));
    // update
    ASSERT(equal(-1.0f,limit.update(-2.0f)));
    ASSERT(equal(1.0f,limit.update(2.0f)));
    ASSERT(equal(0.0f,limit.update(0.0f)));
    printf("BlockLimit:\ttest complete\n");
    return 0;
}

float BlockLimitSym::update(float input)
{
    if (input > getMax())
    {
        input = _max.get();
    }
    else if (input < -getMax())
    {
        input = -getMax();
    }
    return input;
}

int blockLimitSymTest()
{
    BlockLimitSym limit(NULL,"TEST");
    // initial state
    ASSERT(equal(1.0f,limit.getMax()));
    ASSERT(equal(0.0f,limit.getDt()));
    // update
    ASSERT(equal(-1.0f,limit.update(-2.0f)));
    ASSERT(equal(1.0f,limit.update(2.0f)));
    ASSERT(equal(0.0f,limit.update(0.0f)));
    printf("BlockLimitSym:\ttest complete\n");
    return 0;
}

float BlockLowPass::update(float input)
{
    float b = 2*float(M_PI)*getFCut()*getDt();
    float a = b/ (1 + b);
    setState(a*input + (1-a)*getState());
    return getState();
}

int blockLowPassTest()
{
    BlockLowPass lowPass(NULL,"TEST_LP");
    // test initial state
    ASSERT(equal(10.0f,lowPass.getFCut()));
    ASSERT(equal(0.0f,lowPass.getState()));
    ASSERT(equal(0.0f,lowPass.getDt()));
    // set dt
    lowPass.setDt(0.1f);
    ASSERT(equal(0.1f,lowPass.getDt()));
    // set state
    lowPass.setState(1.0f);
    ASSERT(equal(1.0f,lowPass.getState()));
    // test update
    ASSERT(equal(1.8626974f,lowPass.update(2.0f)));
    // test end condition
    for (int i=0;i<100;i++)
    {
        lowPass.update(2.0f);
    }
    ASSERT(equal(2.0f,lowPass.getState()));
    ASSERT(equal(2.0f,lowPass.update(2.0f)));
    printf("BlockLowPass:\ttest complete\n");
    return 0;
};

float BlockHighPass::update(float input)
{
    float b = 2*float(M_PI)*getFCut()*getDt();
    float a = 1/ (1 + b);
    setY(a * (getY() + input - getU()));
    setU(input);
    return getY();
}

int blockHighPassTest()
{
    BlockHighPass highPass(NULL,"TEST_HP");
    // test initial state
    ASSERT(equal(10.0f,highPass.getFCut()));
    ASSERT(equal(0.0f,highPass.getU()));
    ASSERT(equal(0.0f,highPass.getY()));
    ASSERT(equal(0.0f,highPass.getDt()));
    // set dt
    highPass.setDt(0.1f);
    ASSERT(equal(0.1f,highPass.getDt()));
    // set state
    highPass.setU(1.0f);
    ASSERT(equal(1.0f,highPass.getU()));
    highPass.setY(1.0f);
    ASSERT(equal(1.0f,highPass.getY()));
    // test update
    ASSERT(equal(0.2746051f,highPass.update(2.0f)));
    // test end condition
    for (int i=0;i<100;i++)
    {
        highPass.update(2.0f);
    }
    ASSERT(equal(0.0f,highPass.getY()));
    ASSERT(equal(0.0f,highPass.update(2.0f)));
    printf("BlockHighPass:\ttest complete\n");
    return 0;
}

float BlockIntegral::update(float input)
{
    // trapezoidal integration
    setState(_limit.update(getState() + 
                (getState() + input)*getDt()/2));
    return getState();
}

float BlockDerivative::update(float input)
{
    float output = (input - getState())/getDt();
    setState(input);
    return output;
}

} // namespace control
