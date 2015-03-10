/****************************************************************************
 *
 *   Copyright (c) 2013 PX4 Development Team. All rights reserved.
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
 * @file Limits.cpp
 *
 * Limiting / constrain helper functions
 */


#include <math.h>
#include <stdint.h>

#include "Limits.hpp"


namespace math {

#ifndef CONFIG_ARCH_ARM	
#define M_PI_F 3.14159265358979323846f
#endif

float __EXPORT min(float val1, float val2)
{
	return (val1 < val2) ? val1 : val2;
}

int __EXPORT min(int val1, int val2)
{
	return (val1 < val2) ? val1 : val2;
}

unsigned __EXPORT min(unsigned val1, unsigned val2)
{
	return (val1 < val2) ? val1 : val2;
}

uint64_t __EXPORT min(uint64_t val1, uint64_t val2)
{
	return (val1 < val2) ? val1 : val2;
}

double __EXPORT min(double val1, double val2)
{
	return (val1 < val2) ? val1 : val2;
}

float __EXPORT max(float val1, float val2)
{
	return (val1 > val2) ? val1 : val2;
}

int __EXPORT max(int val1, int val2)
{
	return (val1 > val2) ? val1 : val2;
}

unsigned __EXPORT max(unsigned val1, unsigned val2)
{
	return (val1 > val2) ? val1 : val2;
}

uint64_t __EXPORT max(uint64_t val1, uint64_t val2)
{
	return (val1 > val2) ? val1 : val2;
}

double __EXPORT max(double val1, double val2)
{
	return (val1 > val2) ? val1 : val2;
}


float __EXPORT constrain(float val, float min, float max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

int __EXPORT constrain(int val, int min, int max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

unsigned __EXPORT constrain(unsigned val, unsigned min, unsigned max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

uint64_t __EXPORT constrain(uint64_t val, uint64_t min, uint64_t max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

double __EXPORT constrain(double val, double min, double max)
{
	return (val < min) ? min : ((val > max) ? max : val);
}

float __EXPORT radians(float degrees)
{
	return (degrees / 180.0f) * M_PI_F;
}

double __EXPORT radians(double degrees)
{
	return (degrees / 180.0) * M_PI;
}

float __EXPORT degrees(float radians)
{
	return (radians / M_PI_F) * 180.0f;
}

double __EXPORT degrees(double radians)
{
	return (radians / M_PI) * 180.0;
}

}
