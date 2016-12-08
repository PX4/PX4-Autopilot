/****************************************************************************
 *
 *   Copyright (c) 2013, 2014 PX4 Development Team. All rights reserved.
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
 * @file BezierQuad.cpp
 * Bezier function
 *
 * @author Dennis Mannhart <dennis.mannhart@gmail.com>
 *
 */

#include "BezierQuad.hpp"

namespace bezier{

#define GOLDEN_RATIO 1.61803398f //(sqrt(5)-1)/2
#define RESOLUTION 0.0001f  //represents resolution; end criterion for golden section search


void
BezierQuad::setBezier(const matrix::Vector3f &pt0, const matrix::Vector3f &ctrl, const matrix::Vector3f &pt1, float duration){
	_pt0 = pt0;
	_ctrl = ctrl;
	_pt1 = pt1;
	_duration = duration;

}

void
BezierQuad::getBezier(matrix::Vector3f &pt0, matrix::Vector3f &ctrl, matrix::Vector3f &pt1){
	pt0 = _pt0;
	ctrl = _ctrl;
	pt1 = _pt1;
}

void
BezierQuad::getPoint(matrix::Vector3f &point, const float t){
	point = _pt0 * (1 - t/_duration) * (1 - t/_duration) + _ctrl * 2.0f * (1- t/_duration)*t/_duration +   _pt1*t/_duration * t/_duration;
}

void
BezierQuad::getVelocity(matrix::Vector3f &vel, const float t){
	vel = ((_ctrl - _pt0)*_duration + (_pt0 - _ctrl *2.0f + _pt1 )*t )*2.0f/(_duration*_duration);
}

void
BezierQuad::getAcceleration(matrix::Vector3f &acc){
	acc = (_pt0 - _ctrl * 2.0f + _pt1) * 2.0f/(_duration * _duration);
}

void
BezierQuad::getStates(matrix::Vector3f &point, matrix::Vector3f &vel, matrix::Vector3f &acc, const float time){
	getPoint(point, time);
	getVelocity(vel, time);
	getAcceleration(acc);
}

float
BezierQuad::getDistanceSquared(const float t, const matrix::Vector3f &pose){
	/* get point on bezier */
	matrix::Vector3f vec;
	getPoint(vec, t);

	/* get vector from point to pose */
	vec = vec - pose;

	/* norm squared */
	return (vec * vec);

}

void
BezierQuad::getStatesClosest(matrix::Vector3f &point,matrix::Vector3f &vel,matrix::Vector3f &acc, const matrix::Vector3f pose){
	/* get t that corresponds to point closest on bezier point */
	float t = goldenSectionSearch(pose);

	/* get states corresponding to t */
	getStates(point, vel, acc, t);

}

void
BezierQuad::computeBezFromVel(const matrix::Vector3f &ctrl, const matrix::Vector3f &vel0, const matrix::Vector3f &vel1, const float duration){

	/* update bezier points */
	_ctrl = ctrl;
	_duration = duration;
	_pt0 = _ctrl - vel0 * _duration/2.0f;
	_pt1 = _ctrl + vel1 * _duration/2.0f;
}

float
BezierQuad::goldenSectionSearch(const matrix::Vector3f &pose){
	float a, b, c, d;
	a = 0.0f; //represents most left point
	b = _duration * 1.0f; //represents most right point

	c = b - (b - a) / GOLDEN_RATIO;
	d = a + (b - a) / GOLDEN_RATIO;

	while(fabsf(c - d) > RESOLUTION){
			if( getDistanceSquared(c, pose) < getDistanceSquared(d, pose)){
				b = d;
			}else{
				a = c;
			}

			c = b - (b -a)/GOLDEN_RATIO;
			d = a + (b -a)/GOLDEN_RATIO;

	}
	return (b+a)/2.0f;
}

}
