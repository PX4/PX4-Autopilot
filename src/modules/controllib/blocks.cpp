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

#include "blocks.hpp"

namespace control
{

int basicBlocksTest()
{
	blockLimitTest();
	blockLimitSymTest();
	blockLowPassTest();
	blockHighPassTest();
	blockIntegralTest();
	blockIntegralTrapTest();
	blockDerivativeTest();
	blockPTest();
	blockPITest();
	blockPDTest();
	blockPIDTest();
	blockOutputTest();
	blockRandUniformTest();
	blockRandGaussTest();
	return 0;
}

float BlockLimit::update(float input)
{
	if (input > getMax()) {
		input = _max.get();

	} else if (input < getMin()) {
		input = getMin();
	}

	return input;
}

int blockLimitTest()
{
	printf("Test BlockLimit\t\t\t: ");
	BlockLimit limit(NULL, "TEST");
	// initial state
	ASSERT(equal(1.0f, limit.getMax()));
	ASSERT(equal(-1.0f, limit.getMin()));
	ASSERT(equal(0.0f, limit.getDt()));
	// update
	ASSERT(equal(-1.0f, limit.update(-2.0f)));
	ASSERT(equal(1.0f, limit.update(2.0f)));
	ASSERT(equal(0.0f, limit.update(0.0f)));
	printf("PASS\n");
	return 0;
}

float BlockLimitSym::update(float input)
{
	if (input > getMax()) {
		input = _max.get();

	} else if (input < -getMax()) {
		input = -getMax();
	}

	return input;
}

int blockLimitSymTest()
{
	printf("Test BlockLimitSym\t\t: ");
	BlockLimitSym limit(NULL, "TEST");
	// initial state
	ASSERT(equal(1.0f, limit.getMax()));
	ASSERT(equal(0.0f, limit.getDt()));
	// update
	ASSERT(equal(-1.0f, limit.update(-2.0f)));
	ASSERT(equal(1.0f, limit.update(2.0f)));
	ASSERT(equal(0.0f, limit.update(0.0f)));
	printf("PASS\n");
	return 0;
}

float BlockLowPass::update(float input)
{
	if (!isfinite(getState())) {
		setState(input);
	}
	float b = 2 * float(M_PI) * getFCut() * getDt();
	float a = b / (1 + b);
	setState(a * input + (1 - a)*getState());
	return getState();
}

int blockLowPassTest()
{
	printf("Test BlockLowPass\t\t: ");
	BlockLowPass lowPass(NULL, "TEST_LP");
	// test initial state
	ASSERT(equal(10.0f, lowPass.getFCut()));
	ASSERT(equal(0.0f, lowPass.getState()));
	ASSERT(equal(0.0f, lowPass.getDt()));
	// set dt
	lowPass.setDt(0.1f);
	ASSERT(equal(0.1f, lowPass.getDt()));
	// set state
	lowPass.setState(1.0f);
	ASSERT(equal(1.0f, lowPass.getState()));
	// test update
	ASSERT(equal(1.8626974f, lowPass.update(2.0f)));

	// test end condition
	for (int i = 0; i < 100; i++) {
		lowPass.update(2.0f);
	}

	ASSERT(equal(2.0f, lowPass.getState()));
	ASSERT(equal(2.0f, lowPass.update(2.0f)));
	printf("PASS\n");
	return 0;
};

float BlockHighPass::update(float input)
{
	float b = 2 * float(M_PI) * getFCut() * getDt();
	float a = 1 / (1 + b);
	setY(a * (getY() + input - getU()));
	setU(input);
	return getY();
}

int blockHighPassTest()
{
	printf("Test BlockHighPass\t\t: ");
	BlockHighPass highPass(NULL, "TEST_HP");
	// test initial state
	ASSERT(equal(10.0f, highPass.getFCut()));
	ASSERT(equal(0.0f, highPass.getU()));
	ASSERT(equal(0.0f, highPass.getY()));
	ASSERT(equal(0.0f, highPass.getDt()));
	// set dt
	highPass.setDt(0.1f);
	ASSERT(equal(0.1f, highPass.getDt()));
	// set state
	highPass.setU(1.0f);
	ASSERT(equal(1.0f, highPass.getU()));
	highPass.setY(1.0f);
	ASSERT(equal(1.0f, highPass.getY()));
	// test update
	ASSERT(equal(0.2746051f, highPass.update(2.0f)));

	// test end condition
	for (int i = 0; i < 100; i++) {
		highPass.update(2.0f);
	}

	ASSERT(equal(0.0f, highPass.getY()));
	ASSERT(equal(0.0f, highPass.update(2.0f)));
	printf("PASS\n");
	return 0;
}

float BlockIntegral::update(float input)
{
	// trapezoidal integration
	setY(_limit.update(getY() + input * getDt()));
	return getY();
}

int blockIntegralTest()
{
	printf("Test BlockIntegral\t\t: ");
	BlockIntegral integral(NULL, "TEST_I");
	// test initial state
	ASSERT(equal(1.0f, integral.getMax()));
	ASSERT(equal(0.0f, integral.getDt()));
	// set dt
	integral.setDt(0.1f);
	ASSERT(equal(0.1f, integral.getDt()));
	// set Y
	integral.setY(0.9f);
	ASSERT(equal(0.9f, integral.getY()));

	// test exceed max
	for (int i = 0; i < 100; i++) {
		integral.update(1.0f);
	}

	ASSERT(equal(1.0f, integral.update(1.0f)));
	// test exceed min
	integral.setY(-0.9f);
	ASSERT(equal(-0.9f, integral.getY()));

	for (int i = 0; i < 100; i++) {
		integral.update(-1.0f);
	}

	ASSERT(equal(-1.0f, integral.update(-1.0f)));
	// test update
	integral.setY(0.1f);
	ASSERT(equal(0.2f, integral.update(1.0)));
	ASSERT(equal(0.2f, integral.getY()));
	printf("PASS\n");
	return 0;
}

float BlockIntegralTrap::update(float input)
{
	// trapezoidal integration
	setY(_limit.update(getY() +
			   (getU() + input) / 2.0f * getDt()));
	setU(input);
	return getY();
}

int blockIntegralTrapTest()
{
	printf("Test BlockIntegralTrap\t\t: ");
	BlockIntegralTrap integral(NULL, "TEST_I");
	// test initial state
	ASSERT(equal(1.0f, integral.getMax()));
	ASSERT(equal(0.0f, integral.getDt()));
	// set dt
	integral.setDt(0.1f);
	ASSERT(equal(0.1f, integral.getDt()));
	// set U
	integral.setU(1.0f);
	ASSERT(equal(1.0f, integral.getU()));
	// set Y
	integral.setY(0.9f);
	ASSERT(equal(0.9f, integral.getY()));

	// test exceed max
	for (int i = 0; i < 100; i++) {
		integral.update(1.0f);
	}

	ASSERT(equal(1.0f, integral.update(1.0f)));
	// test exceed min
	integral.setU(-1.0f);
	integral.setY(-0.9f);
	ASSERT(equal(-0.9f, integral.getY()));

	for (int i = 0; i < 100; i++) {
		integral.update(-1.0f);
	}

	ASSERT(equal(-1.0f, integral.update(-1.0f)));
	// test update
	integral.setU(2.0f);
	integral.setY(0.1f);
	ASSERT(equal(0.25f, integral.update(1.0)));
	ASSERT(equal(0.25f, integral.getY()));
	ASSERT(equal(1.0f, integral.getU()));
	printf("PASS\n");
	return 0;
}

float BlockDerivative::update(float input)
{
	float output;
	if (_initialized) {
		output = _lowPass.update((input - getU()) / getDt());
	} else {
		// if this is the first call to update
		// we have no valid derivative
		// and so we use the assumption the
		// input value is not changing much,
		// which is the best we can do here.
		output = 0.0f;
		_initialized = true;
	}
	setU(input);
	return output;
}

int blockDerivativeTest()
{
	printf("Test BlockDerivative\t\t: ");
	BlockDerivative derivative(NULL, "TEST_D");
	// test initial state
	ASSERT(equal(0.0f, derivative.getU()));
	ASSERT(equal(10.0f, derivative.getLP()));
	// set dt
	derivative.setDt(0.1f);
	ASSERT(equal(0.1f, derivative.getDt()));
	// set U
	derivative.setU(1.0f);
	ASSERT(equal(1.0f, derivative.getU()));
	// test  update
	ASSERT(equal(8.6269744f, derivative.update(2.0f)));
	ASSERT(equal(2.0f, derivative.getU()));
	printf("PASS\n");
	return 0;
}

int blockPTest()
{
	printf("Test BlockP\t\t\t: ");
	BlockP blockP(NULL, "TEST_P");
	// test initial state
	ASSERT(equal(0.2f, blockP.getKP()));
	ASSERT(equal(0.0f, blockP.getDt()));
	// set dt
	blockP.setDt(0.1f);
	ASSERT(equal(0.1f, blockP.getDt()));
	// test  update
	ASSERT(equal(0.4f, blockP.update(2.0f)));
	printf("PASS\n");
	return 0;
}

int blockPITest()
{
	printf("Test BlockPI\t\t\t: ");
	BlockPI blockPI(NULL, "TEST");
	// test initial state
	ASSERT(equal(0.2f, blockPI.getKP()));
	ASSERT(equal(0.1f, blockPI.getKI()));
	ASSERT(equal(0.0f, blockPI.getDt()));
	ASSERT(equal(1.0f, blockPI.getIntegral().getMax()));
	// set dt
	blockPI.setDt(0.1f);
	ASSERT(equal(0.1f, blockPI.getDt()));
	// set integral state
	blockPI.getIntegral().setY(0.1f);
	ASSERT(equal(0.1f, blockPI.getIntegral().getY()));
	// test  update
	// 0.2*2 + 0.1*(2*0.1 + 0.1) = 0.43
	ASSERT(equal(0.43f, blockPI.update(2.0f)));
	printf("PASS\n");
	return 0;
}

int blockPDTest()
{
	printf("Test BlockPD\t\t\t: ");
	BlockPD blockPD(NULL, "TEST");
	// test initial state
	ASSERT(equal(0.2f, blockPD.getKP()));
	ASSERT(equal(0.01f, blockPD.getKD()));
	ASSERT(equal(0.0f, blockPD.getDt()));
	ASSERT(equal(10.0f, blockPD.getDerivative().getLP()));
	// set dt
	blockPD.setDt(0.1f);
	ASSERT(equal(0.1f, blockPD.getDt()));
	// set derivative state
	blockPD.getDerivative().setU(1.0f);
	ASSERT(equal(1.0f, blockPD.getDerivative().getU()));
	// test  update
	// 0.2*2 + 0.1*(0.1*8.626...) = 0.486269744
	ASSERT(equal(0.486269744f, blockPD.update(2.0f)));
	printf("PASS\n");
	return 0;
}

int blockPIDTest()
{
	printf("Test BlockPID\t\t\t: ");
	BlockPID blockPID(NULL, "TEST");
	// test initial state
	ASSERT(equal(0.2f, blockPID.getKP()));
	ASSERT(equal(0.1f, blockPID.getKI()));
	ASSERT(equal(0.01f, blockPID.getKD()));
	ASSERT(equal(0.0f, blockPID.getDt()));
	ASSERT(equal(10.0f, blockPID.getDerivative().getLP()));
	ASSERT(equal(1.0f, blockPID.getIntegral().getMax()));
	// set dt
	blockPID.setDt(0.1f);
	ASSERT(equal(0.1f, blockPID.getDt()));
	// set derivative state
	blockPID.getDerivative().setU(1.0f);
	ASSERT(equal(1.0f, blockPID.getDerivative().getU()));
	// set integral state
	blockPID.getIntegral().setY(0.1f);
	ASSERT(equal(0.1f, blockPID.getIntegral().getY()));
	// test  update
	// 0.2*2 + 0.1*(2*0.1 + 0.1) + 0.1*(0.1*8.626...) = 0.5162697
	ASSERT(equal(0.5162697f, blockPID.update(2.0f)));
	printf("PASS\n");
	return 0;
}

int blockOutputTest()
{
	printf("Test BlockOutput\t\t: ");
	BlockOutput blockOutput(NULL, "TEST");
	// test initial state
	ASSERT(equal(0.0f, blockOutput.getDt()));
	ASSERT(equal(0.5f, blockOutput.get()));
	ASSERT(equal(-1.0f, blockOutput.getMin()));
	ASSERT(equal(1.0f, blockOutput.getMax()));
	// test update below min
	blockOutput.update(-2.0f);
	ASSERT(equal(-1.0f, blockOutput.get()));
	// test update above max
	blockOutput.update(2.0f);
	ASSERT(equal(1.0f, blockOutput.get()));
	// test trim
	blockOutput.update(0.0f);
	ASSERT(equal(0.5f, blockOutput.get()));
	printf("PASS\n");
	return 0;
}

int blockRandUniformTest()
{
	srand(1234);
	printf("Test BlockRandUniform\t\t: ");
	BlockRandUniform blockRandUniform(NULL, "TEST");
	// test initial state
	ASSERT(equal(0.0f, blockRandUniform.getDt()));
	ASSERT(equal(-1.0f, blockRandUniform.getMin()));
	ASSERT(equal(1.0f, blockRandUniform.getMax()));
	// test update
	int n = 10000;
	float mean = blockRandUniform.update();

	// recursive mean algorithm from Knuth
	for (int i = 2; i < n + 1; i++) {
		float val = blockRandUniform.update();
		mean += (val - mean) / i;
		ASSERT(val <= blockRandUniform.getMax());
		ASSERT(val >= blockRandUniform.getMin());
	}

	ASSERT(equal(mean, (blockRandUniform.getMin() +
			    blockRandUniform.getMax()) / 2, 1e-1));
	printf("PASS\n");
	return 0;
}

int blockRandGaussTest()
{
	srand(1234);
	printf("Test BlockRandGauss\t\t: ");
	BlockRandGauss blockRandGauss(NULL, "TEST");
	// test initial state
	ASSERT(equal(0.0f, blockRandGauss.getDt()));
	ASSERT(equal(1.0f, blockRandGauss.getMean()));
	ASSERT(equal(2.0f, blockRandGauss.getStdDev()));
	// test update
	int n = 10000;
	float mean = blockRandGauss.update();
	float sum = 0;

	// recursive mean, stdev algorithm from Knuth
	for (int i = 2; i < n + 1; i++) {
		float val = blockRandGauss.update();
		float newMean = mean + (val - mean) / i;
		sum += (val - mean) * (val - newMean);
		mean = newMean;
	}

	float stdDev = sqrt(sum / (n - 1));
	ASSERT(equal(mean, blockRandGauss.getMean(), 1e-1));
	ASSERT(equal(stdDev, blockRandGauss.getStdDev(), 1e-1));
	printf("PASS\n");
	return 0;
}

} // namespace control
